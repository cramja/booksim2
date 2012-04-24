// $Id$

/*
 Copyright (c) 2007-2011, Trustees of The Leland Stanford Junior University
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this 
 list of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice, this
 list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*buffer_state.cpp
 *
 * This class is the buffere state of the next router down the channel
 * tracks the credit and how much of the buffer is in use 
 */

#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cassert>
#include <limits>

#include "booksim.hpp"
#include "buffer_state.hpp"
#include "random_utils.hpp"
#include "globals.hpp"

//#define DEBUG_FEEDBACK
//#define DEBUG_SIMPLEFEEDBACK

BufferState::BufferPolicy::BufferPolicy(Configuration const & config, BufferState * parent, const string & name)
: Module(parent, name), _buffer_state(parent)
{
}

void BufferState::BufferPolicy::TakeBuffer(int vc) {
}

void BufferState::BufferPolicy::SendingFlit(Flit const * const f) {
}

void BufferState::BufferPolicy::FreeSlotFor(int vc) {
}

BufferState::BufferPolicy * BufferState::BufferPolicy::New(Configuration const & config, BufferState * parent, const string & name)
{
  BufferPolicy * sp = NULL;
  string buffer_policy = config.GetStr("buffer_policy");
  if(buffer_policy == "private") {
    sp = new PrivateBufferPolicy(config, parent, name);
  } else if(buffer_policy == "shared") {
    sp = new SharedBufferPolicy(config, parent, name);
  } else if(buffer_policy == "limited") {
    sp = new LimitedSharedBufferPolicy(config, parent, name);
  } else if(buffer_policy == "dynamic") {
    sp = new DynamicLimitedSharedBufferPolicy(config, parent, name);
  } else if(buffer_policy == "shifting") {
    sp = new ShiftingDynamicLimitedSharedBufferPolicy(config, parent, name);
  } else if(buffer_policy == "feedback") {
    sp = new FeedbackSharedBufferPolicy(config, parent, name);
  } else if(buffer_policy == "simplefeedback") {
    sp = new SimpleFeedbackSharedBufferPolicy(config, parent, name);
  } else {
    cout << "Unknown buffer policy: " << buffer_policy << endl;
  }
  return sp;
}

BufferState::PrivateBufferPolicy::PrivateBufferPolicy(Configuration const & config, BufferState * parent, const string & name)
  : BufferPolicy(config, parent, name)
{
  int const vcs = config.GetInt( "num_vcs" );
  int const buf_size = config.GetInt("buf_size");
  if(buf_size <= 0) {
    _vc_buf_size = config.GetInt("vc_buf_size");
  } else {
    _vc_buf_size = buf_size / vcs;
  }
  assert(_vc_buf_size > 0);
}

void BufferState::PrivateBufferPolicy::SendingFlit(Flit const * const f)
{
  int const vc = f->vc;
  if(_buffer_state->Occupancy(vc) > _vc_buf_size) {
    ostringstream err;
    err << "Buffer overflow for VC " << vc;
    Error(err.str());
  }
}

bool BufferState::PrivateBufferPolicy::IsFullFor(int vc) const
{
  return (_buffer_state->Occupancy(vc) >= _vc_buf_size);
}

BufferState::SharedBufferPolicy::SharedBufferPolicy(Configuration const & config, BufferState * parent, const string & name)
  : BufferPolicy(config, parent, name), _shared_buf_occupancy(0)
{
  int const vcs = config.GetInt( "num_vcs" );
  int num_private_bufs = config.GetInt("private_bufs");
  if(num_private_bufs < 0) {
    num_private_bufs = vcs;
  } else if(num_private_bufs == 0) {
    num_private_bufs = 1;
  }
  
  _private_buf_occupancy.resize(num_private_bufs, 0);

  _buf_size = config.GetInt("buf_size");
  if(_buf_size < 0) {
    _buf_size = vcs * config.GetInt("vc_buf_size");
  }

  _private_buf_size = config.GetIntArray("private_buf_size");
  if(_private_buf_size.empty()) {
    int const bs = config.GetInt("private_buf_size");
    if(bs < 0) {
      _private_buf_size.push_back(_buf_size / num_private_bufs);
    } else {
      _private_buf_size.push_back(bs);
    }
  }
  _private_buf_size.resize(num_private_bufs, _private_buf_size.back());
  
  vector<int> start_vc = config.GetIntArray("private_buf_start_vc");
  if(start_vc.empty()) {
    int const sv = config.GetInt("private_buf_start_vc");
    if(sv < 0) {
      start_vc.resize(num_private_bufs);
      for(int i = 0; i < num_private_bufs; ++i) {
	start_vc[i] = i * vcs / num_private_bufs;
      }
    } else {
      start_vc.push_back(sv);
    }
  }
  
  vector<int> end_vc = config.GetIntArray("private_buf_end_vc");
  if(end_vc.empty()) {
    int const ev = config.GetInt("private_buf_end_vc");
    if(ev < 0) {
      end_vc.resize(num_private_bufs);
      for(int i = 0; i < num_private_bufs; ++i) {
	end_vc[i] = (i + 1) * vcs / num_private_bufs - 1;
      }
    } else {
      end_vc.push_back(ev);
    }
  }

  _private_buf_vc_map.resize(vcs, -1);
  _shared_buf_size = _buf_size;
  for(int i = 0; i < num_private_bufs; ++i) {
    _shared_buf_size -= _private_buf_size[i];
    assert(start_vc[i] <= end_vc[i]);
    for(int v = start_vc[i]; v <= end_vc[i]; ++v) {
      assert(_private_buf_vc_map[v] < 0);
      _private_buf_vc_map[v] = i;
    }
  }
  assert(_shared_buf_size >= 0);

  _reserved_slots.resize(vcs, 0);
}

void BufferState::SharedBufferPolicy::ProcessFreeSlot(int vc)
{
  int i = _private_buf_vc_map[vc];
  --_private_buf_occupancy[i];
  if(_private_buf_occupancy[i] < 0) {
    ostringstream err;
    err << "Private buffer occupancy fell below zero for buffer " << i;
    Error(err.str());
  } else if(_private_buf_occupancy[i] >= _private_buf_size[i]) {
    --_shared_buf_occupancy;
    if(_shared_buf_occupancy < 0) {
      Error("Shared buffer occupancy fell below zero.");
    }
  }
}

void BufferState::SharedBufferPolicy::SendingFlit(Flit const * const f)
{
  int const vc = f->vc;
  if(_reserved_slots[vc] > 0) {
    --_reserved_slots[vc];
  } else {
    int i = _private_buf_vc_map[vc];
    ++_private_buf_occupancy[i];
    if(_private_buf_occupancy[i] > _private_buf_size[i]) {
      ++_shared_buf_occupancy;
      if(_shared_buf_occupancy > _shared_buf_size) {
	Error("Shared buffer overflow.");
      }
    }
  }
  if(f->tail) {
    while(_reserved_slots[vc]) {
      --_reserved_slots[vc];
      ProcessFreeSlot(vc);
    }
  }
}

void BufferState::SharedBufferPolicy::FreeSlotFor(int vc)
{
  if(!_buffer_state->IsAvailableFor(vc) && _buffer_state->IsEmptyFor(vc)) {
    ++_reserved_slots[vc];
  } else {
    ProcessFreeSlot(vc);
  }
}

bool BufferState::SharedBufferPolicy::IsFullFor(int vc) const
{
  int i = _private_buf_vc_map[vc];
  return ((_reserved_slots[vc] == 0) &&
	  (_private_buf_occupancy[i] >= _private_buf_size[i]) &&
	  (_shared_buf_occupancy >= _shared_buf_size));
}

BufferState::LimitedSharedBufferPolicy::LimitedSharedBufferPolicy(Configuration const & config, BufferState * parent, const string & name)
  : SharedBufferPolicy(config, parent, name), _active_vcs(0)
{
  _vcs = config.GetInt("num_vcs");
  _max_held_slots = config.GetInt("max_held_slots");
  if(_max_held_slots < 0) {
    _max_held_slots = _buf_size;
  }
}

void BufferState::LimitedSharedBufferPolicy::TakeBuffer(int vc)
{
  ++_active_vcs;
  if(_active_vcs > _vcs) {
    Error("Number of active VCs is too large.");
  }
}

void BufferState::LimitedSharedBufferPolicy::SendingFlit(Flit const * const f)
{
  SharedBufferPolicy::SendingFlit(f);
  if(f->tail) {
    --_active_vcs;
    if(_active_vcs < 0) {
      Error("Number of active VCs fell below zero.");
    }
  }
}

bool BufferState::LimitedSharedBufferPolicy::IsFullFor(int vc) const
{
  return (SharedBufferPolicy::IsFullFor(vc) ||
	  (_buffer_state->Occupancy(vc) >= _max_held_slots));
}

BufferState::DynamicLimitedSharedBufferPolicy::DynamicLimitedSharedBufferPolicy(Configuration const & config, BufferState * parent, const string & name)
  : LimitedSharedBufferPolicy(config, parent, name)
{
  _max_held_slots = _buf_size;
}

void BufferState::DynamicLimitedSharedBufferPolicy::TakeBuffer(int vc)
{
  LimitedSharedBufferPolicy::TakeBuffer(vc);
  assert(_active_vcs > 0);
  _max_held_slots = _buf_size / _active_vcs;
  assert(_max_held_slots > 0);
}

void BufferState::DynamicLimitedSharedBufferPolicy::SendingFlit(Flit const * const f)
{
  LimitedSharedBufferPolicy::SendingFlit(f);
  if(f->tail && _active_vcs) {
    _max_held_slots = _buf_size / _active_vcs;
  }
  assert(_max_held_slots > 0);
}

BufferState::ShiftingDynamicLimitedSharedBufferPolicy::ShiftingDynamicLimitedSharedBufferPolicy(Configuration const & config, BufferState * parent, const string & name)
  : DynamicLimitedSharedBufferPolicy(config, parent, name)
{

}

void BufferState::ShiftingDynamicLimitedSharedBufferPolicy::TakeBuffer(int vc)
{
  LimitedSharedBufferPolicy::TakeBuffer(vc);
  assert(_active_vcs);
  int i = _active_vcs - 1;
  _max_held_slots = _buf_size;
  while(i) {
    _max_held_slots >>= 1;
    i >>= 1;
  }
  assert(_max_held_slots > 0);
}

void BufferState::ShiftingDynamicLimitedSharedBufferPolicy::SendingFlit(Flit const * const f)
{
  LimitedSharedBufferPolicy::SendingFlit(f);
  if(f->tail && _active_vcs) {
    int i = _active_vcs - 1;
    _max_held_slots = _buf_size;
    while(i) {
      _max_held_slots >>= 1;
      i >>= 1;
    }
  }
  assert(_max_held_slots > 0);
}

BufferState::FeedbackSharedBufferPolicy::FeedbackSharedBufferPolicy(Configuration const & config, BufferState * parent, const string & name)
  : SharedBufferPolicy(config, parent, name)
{
  _aging_scale = config.GetInt("feedback_aging_scale");
  _offset = config.GetInt("feedback_offset");
  _vcs = config.GetInt("num_vcs");

  _occupancy_limit.resize(_vcs, _buf_size);
  _round_trip_time.resize(_vcs, -1);
  _flit_sent_time.resize(_vcs);
  _total_mapped_size = _buf_size * _vcs;
  _min_round_trip_time = numeric_limits<int>::max();
}

void BufferState::FeedbackSharedBufferPolicy::SendingFlit(Flit const * const f)
{
  SharedBufferPolicy::SendingFlit(f);
  _flit_sent_time[f->vc].push(GetSimTime());
}

void BufferState::FeedbackSharedBufferPolicy::FreeSlotFor(int vc)
{
  SharedBufferPolicy::FreeSlotFor(vc);
  assert(!_flit_sent_time[vc].empty());
  int const last_rtt = GetSimTime() - _flit_sent_time[vc].front();
#ifdef DEBUG_FEEDBACK
  cerr << FullName() << ": Probe for VC "
       << vc << " came back after "
       << last_rtt << " cycles."
       << endl;
#endif
  _flit_sent_time[vc].pop();
  
  // determine minimum round trip time (could be hardcoded in a real network, 
  // but since some of the topologies here have varying channel lengths, it's 
  // easiest just to detect this on the fly)
  if(last_rtt < _min_round_trip_time) {
    _min_round_trip_time = last_rtt;
#ifdef DEBUG_FEEDBACK
    cerr << FullName() << ": Updating minimum RTT to "
	 << last_rtt << " cycles."
	 << endl;
#endif
  }

  // update moving average of round-trip time
  int rtt = _round_trip_time[vc];
  if(rtt < 0) {
    rtt = last_rtt;
  } else {
    rtt = ((rtt << _aging_scale) + last_rtt - rtt) >> _aging_scale;
  }
#ifdef DEBUG_FEEDBACK
  cerr << FullName() << ": Updating RTT estimate for VC "
       << vc << " to "
       << rtt << " cycles."
       << endl;
#endif
  _round_trip_time[vc] = rtt;
  
  // update occupancy limit for this VC
  // 
  // for every cycle that the measured average round trip time exceeded the 
  // observed minimum round trip time, reduce buffer occupancy limit by one
  int limit = _occupancy_limit[vc];
  _total_mapped_size -= limit;
  limit = max((_min_round_trip_time << 1) - rtt + _offset, 1);
  _occupancy_limit[vc] = limit;
  _total_mapped_size += limit;  
#ifdef DEBUG_FEEDBACK
  cerr << FullName() << ": Occupancy limit for VC "
       << vc << " is "
       << limit << " slots."
       << endl;
  cerr << FullName() << ": Total mapped buffer space is "
       << _total_mapped_size << " slots."
       << endl;
#endif
}

bool BufferState::FeedbackSharedBufferPolicy::IsFullFor(int vc) const
{
  return (SharedBufferPolicy::IsFullFor(vc) ||
	  (_buffer_state->Occupancy(vc) >= _occupancy_limit[vc]));
}

BufferState::SimpleFeedbackSharedBufferPolicy::SimpleFeedbackSharedBufferPolicy(Configuration const & config, BufferState * parent, const string & name)
  : FeedbackSharedBufferPolicy(config, parent, name)
{
  _pending_credits.resize(_vcs, 0);
}

void BufferState::SimpleFeedbackSharedBufferPolicy::SendingFlit(Flit const * const f)
{
  int const & vc = f->vc;
  if(_flit_sent_time[vc].empty()) {
    _pending_credits[vc] = _buffer_state->Occupancy(vc) - 1;
#ifdef DEBUG_SIMPLEFEEDBACK
    cerr << FullName() << ": Sending probe flit for VC "
	 << vc << "; "
	 << _pending_credits[vc] << " non-probe flits in flight."
	 << endl;
#endif
    FeedbackSharedBufferPolicy::SendingFlit(f);
    return;
  }
  SharedBufferPolicy::SendingFlit(f);
}

void BufferState::SimpleFeedbackSharedBufferPolicy::FreeSlotFor(int vc)
{
  if(!_flit_sent_time[vc].empty() && _pending_credits[vc] == 0) {
#ifdef DEBUG_SIMPLEFEEDBACK
    cerr << FullName() << ": Probe credit for VC "
	 << vc << " came back." << endl;
#endif
    FeedbackSharedBufferPolicy::FreeSlotFor(vc);
    return;
  }
  if(_pending_credits[vc] > 0) {
    assert(!_flit_sent_time[vc].empty());
    --_pending_credits[vc];
#ifdef DEBUG_SIMPLEFEEDBACK
    cerr << FullName() << ": Ignoring non-probe credit for VC "
	 << vc << "; "
	 << _pending_credits[vc] << " remaining."
	 << endl;
#endif
  }
  SharedBufferPolicy::FreeSlotFor(vc);
}

BufferState::BufferState( const Configuration& config, Module *parent, const string& name ) : 
  Module( parent, name ), _occupancy(0)
{
  _vcs = config.GetInt( "num_vcs" );
  _size = config.GetInt("buf_size");
  if(_size < 0) {
    _size = _vcs * config.GetInt("vc_buf_size");
  }

  _buffer_policy = BufferPolicy::New(config, this, "policy");

  _wait_for_tail_credit = config.GetInt( "wait_for_tail_credit" );

  _vc_occupancy.resize(_vcs, 0);

  _in_use.resize(_vcs, false);
  _tail_sent.resize(_vcs, false);

  _last_id.resize(_vcs, -1);
  _last_pid.resize(_vcs, -1);

#ifdef TRACK_BUFFERS
  _classes = config.GetInt("classes");
  _outstanding_classes.resize(_vcs);
  _class_occupancy.resize(_classes, 0);
#endif
}

BufferState::~BufferState()
{
  delete _buffer_policy;
}

void BufferState::ProcessCredit( Credit const * const c )
{
  assert( c );

  set<int>::iterator iter = c->vc.begin();
  while(iter != c->vc.end()) {

    int const vc = *iter;

    assert( ( vc >= 0 ) && ( vc < _vcs ) );

    if ( ( _wait_for_tail_credit ) && 
	 ( !_in_use[vc] ) ) {
      ostringstream err;
      err << "Received credit for idle VC " << vc;
      Error( err.str() );
    }
    --_occupancy;
    if(_occupancy < 0) {
      Error("Buffer occupancy fell below zero.");
    }
    --_vc_occupancy[vc];
    if(_vc_occupancy[vc] < 0) {
      ostringstream err;
      err << "Buffer occupancy fell below zero for VC " << vc;
      Error(err.str());
    }
    if(_wait_for_tail_credit && !_vc_occupancy[vc] && _tail_sent[vc]) {
      assert(_in_use[vc]);
      _in_use[vc] = false;
    }

#ifdef TRACK_BUFFERS
    assert(!_outstanding_classes[vc].empty());
    int cl = _outstanding_classes[vc].front();
    _outstanding_classes[vc].pop();
    assert((cl >= 0) && (cl < _classes));
    assert(_class_occupancy[cl] > 0);
    --_class_occupancy[cl];
#endif

    _buffer_policy->FreeSlotFor(vc);

    ++iter;
  }
}


void BufferState::SendingFlit( Flit const * const f )
{
  int const vc = f->vc;

  assert( f && ( vc >= 0 ) && ( vc < _vcs ) );

  ++_occupancy;
  if(_occupancy > _size) {
    Error("Buffer overflow.");
  }

  ++_vc_occupancy[vc];
  
  _buffer_policy->SendingFlit(f);
  
#ifdef TRACK_BUFFERS
  _outstanding_classes[vc].push(f->cl);
  ++_class_occupancy[f->cl];
#endif

  if ( f->tail ) {
    _tail_sent[vc] = true;
    
    if ( !_wait_for_tail_credit ) {
      assert(_in_use[vc]);
      _in_use[vc] = false;
    }
  }
  _last_id[vc] = f->id;
  _last_pid[vc] = f->pid;
}

void BufferState::TakeBuffer( int vc )
{
  assert( ( vc >= 0 ) && ( vc < _vcs ) );

  if ( _in_use[vc] ) {
    ostringstream err;
    err << "Buffer taken while in use for VC " << vc;
    Error( err.str() );
  }
  _in_use[vc]    = true;
  _tail_sent[vc] = false;
  _buffer_policy->TakeBuffer(vc);
}

void BufferState::Display( ostream & os ) const
{
  os << FullName() << " :" << endl;
  os << " occupied = " << _occupancy << endl;
  for ( int v = 0; v < _vcs; ++v ) {
    os << "  VC " << v << ": ";
    os << "in_use = " << _in_use[v] 
       << ", tail_sent = " << _tail_sent[v]
       << ", occupied = " << _vc_occupancy[v] << endl;
  }
}
