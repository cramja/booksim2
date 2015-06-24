/*
  Copyright (c) 2007-2015, Trustees of The Leland Stanford Junior University
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  Redistributions of source code must retain the above copyright notice, this list
  of conditions and the following disclaimer.
  Redistributions in binary form must reproduce the above copyright notice, this 
  list of conditions and the following disclaimer in the documentation and/or 
  other materials provided with the distribution.
  Neither the name of the Stanford University nor the names of its contributors 
  may be used to endorse or promote products derived from this software without 
  specific prior written permission.

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

#include "booksim.hpp"
#include <vector>
#include <sstream>

#include "dragonflyrelative.hpp"
#include "random_utils.hpp"
#include "misc_utils.hpp"
#include "globals.hpp"

#define DRAGON_LATENCY

int gPP, gAA, gGG;

//calculate the hop count between src and estination
int dragonflyrelative_hopcnt(int src, int dest) 
{
  int hopcnt;
  int dest_grp_ID, src_grp_ID; 
  int src_hopcnt, dest_hopcnt;
  int src_intm, dest_intm;
  int grp_output, dest_grp_output;
  int grp_output_RID;

  int _grp_num_routers= gAA;
  int _grp_num_nodes =_grp_num_routers*gPP;
  
  dest_grp_ID = int(dest/_grp_num_nodes);
  src_grp_ID = int(src / _grp_num_nodes);
  
  //source and dest are in the same group, either 0-1 hop
  if (dest_grp_ID == src_grp_ID) {
    if ((int)(dest / gPP) == (int)(src /gPP))
      hopcnt = 0;
    else
      hopcnt = 1;
    
  } else {
    //source and dest are in the same group
    //find the number of hops in the source group
    //find the number of hops in the dest group
    if (src_grp_ID > dest_grp_ID)  {
      grp_output = dest_grp_ID;
      dest_grp_output = src_grp_ID - 1;
    }
    else {
      grp_output = dest_grp_ID - 1;
      dest_grp_output = src_grp_ID;
    }
    grp_output_RID = ((int) (grp_output / (gPP))) + src_grp_ID * _grp_num_routers;
    src_intm = grp_output_RID * gPP;

    grp_output_RID = ((int) (dest_grp_output / (gPP))) + dest_grp_ID * _grp_num_routers;
    dest_intm = grp_output_RID * gPP;

    //hop count in source group
    if ((int)( src_intm / gPP) == (int)( src / gPP ) )
      src_hopcnt = 0;
    else
      src_hopcnt = 1; 

    //hop count in destination group
    if ((int)( dest_intm / gPP) == (int)( dest / gPP ) ){
      dest_hopcnt = 0;
    }else{
      dest_hopcnt = 1;
    }

    //tally
    hopcnt = src_hopcnt + 1 + dest_hopcnt;
  }

  return hopcnt;  
}

// packet output port based on the source, destination and current location
// never to be called if you are on the dest router
// MS: it seems as though the returned value is the offset to the global
//     port in the current router's channel array
int dragonflyrelative_port(int rID, int source, int dest, bool debug) {
  int _grp_num_routers= gAA;
  int _grp_num_nodes =_grp_num_routers*gPP;

  int out_port = -1;
  int grp_ID = int(rID / _grp_num_routers); 
  int dest_grp_ID = int(dest/_grp_num_nodes);
  //int grp_output=-1;
  int grp_RID=-1;

  int dist = (( dest_grp_ID + gGG ) - grp_ID ) % gGG; // forward distance

  // which router within this group the packet needs to go to
  // Node channels are first
  if (dest_grp_ID == grp_ID) {
    grp_RID = (dest % _grp_num_nodes) / gPP;  // router with the dest node
  } else {
    grp_RID = (dist - 1) / gPP; // router with the global link to our hop
  }

  if(debug){
    cout << "\nMessage\n";
    cout << "dest group id: " << dest_grp_ID << "\n";
    cout << "group id: " << grp_ID << "\n";
    cout << "grp_RID: " << grp_RID << "\n";
    cout << "dist: " << dist << "\n";
  }
  
  if (dest >= rID*gPP && dest < (rID+1)*gPP) {
    // at the last hop  
    out_port = dest % gPP; // channel to the node
  } else if (grp_RID == rID % gAA) {
    // at the optical link
    out_port = gPP + (gAA-1) + ( (dist - 1) % gPP );
  } else {
    // need to route within a group
    assert(grp_RID!=-1);

    if (rID < grp_RID){
      out_port = (grp_RID % _grp_num_routers) - 1 + gPP;
    }else{
      out_port = (grp_RID % _grp_num_routers) + gPP;
    }
  }  
 
  assert(out_port!=-1);
  if(debug){
    cout << "source: " << source << "\n";
    cout << "dest: " << dest << "\n";
    cout << "router: " << rID << "\n";
    cout << "returned port: " << out_port << "\n";
    getchar();
  }

  return out_port;
}

int dragonflyrelative_port(int rID, int source, int dest) {
  return dragonflyrelative_port(rID, source, dest, false);
}

DragonFlyRelative::DragonFlyRelative( const Configuration &config, const string & name ) :
  Network( config, name )
{

  _ComputeSize( config );
  _Alloc( );
  _BuildNet( config );
}

void DragonFlyRelative::_ComputeSize( const Configuration &config )
{

  // LIMITATION
  //  -- only one dimension between the group
  // _n == # of dimensions within a group
  // _p == # of processors within a router
  // inter-group ports : _p
  // terminal ports : _p
  // intra-group ports : 2*_p - 1
  _p = config.GetInt( "k" );	// # of ports in each switch
  _n = config.GetInt( "n" );


  assert(_n==1);
  // dimension

  if (_n == 1)
    _k = _p + _p + 2*_p  - 1;
  else
    _k = _p + _p + 2*_p;

  
  // FIX...
  gK = _p; gN = _n;

  // with 1 dimension, total of 2p routers per group
  // N = 2p * p * (2p^2 + 1)
  // a = # of routers per group
  //   = 2p (if n = 1)
  //   = p^(n) (if n > 2)
  //  g = # of groups
  //    = a * p + 1
  // N = a * p * g;
  
  if (_n == 1)
    _a = 2 * _p;
  else
    _a = powi(_p, _n);

  _g = _a * _p + 1;
  _nodes   = _a * _p * _g;

  _num_of_switch = _nodes / _p;
  _channels = _num_of_switch * (_k - _p); 
  _size = _num_of_switch;


  
  gGG = _g;
  gPP = _p;
  gAA = _a;
  _grp_num_routers = gAA;
  _grp_num_nodes =_grp_num_routers*gPP;

}

void DragonFlyRelative::_BuildNet( const Configuration &config )
{
  int _output=-1;
  int _input=-1;
  int _dim_ID=-1;
  int _num_ports_per_switch=-1;
  int c;

  ostringstream router_name;



  cout << " Dragonfly Relative" << endl;
  cout << " processors per router = " << _p << " dimension = " << _n << endl; // MS: _p - processors per router, _n - dimension, always 1
  cout << " each router has radix =  "<< _k << endl;
  cout << " # of routers = "<<  _num_of_switch << endl;
  cout << " # of channels = "<<  _channels << endl;
  cout << " # of nodes (processors) = " << _nodes << endl;
  cout << " # of groups = " << _g << endl;
  cout << " # of routers per group = " << _a << endl;

  // MS: loop through each router. this is confusing because they call it node
  // which is not the terminology which we have been using
  for ( int node = 0; node < _num_of_switch; ++node ) {
    // ID of the group
    int grp_ID;
    grp_ID = (int) (node/_a);
    router_name << "router";
    
    router_name << "_" <<  node ;

    _routers[node] = Router::NewRouter( config, this, router_name.str(), node, _k, _k );
    _timed_modules.push_back(_routers[node]);

    router_name.str("");

    // add total input channels
    // MS: _inject contains processors which 'inject' packets
    for ( int cnt = 0; cnt < _p; ++cnt ) {
      c = _p * node +  cnt;
      // MS: network stores all injection channels, retrieve from the network
      _routers[node]->AddInputChannel( _inject[c], _inject_cred[c] ); 

    }

    // add total output channels
    // MS: eject contains 
    for ( int cnt = 0; cnt < _p; ++cnt ) {
      c = _p * node +  cnt;
      _routers[node]->AddOutputChannel( _eject[c], _eject_cred[c] );

    }

    // MS: I changed the following comment. Previously, _p was _k, the radix
    //     which, to my knowledge was incorrect. They use _p in the loop

    // add OUTPUT channels
    // _p == # of processor per router
    //  need 2*_p routers per group
    //  MS: 2*_p-1 output channels per router for local connections
    //  MS: _p outputs for intergroup

    if (_n > 1 )  { cout << " ERROR: n>1 dimension NOT supported yet... " << endl; exit(-1); }

    //********************************************
    //   connect OUTPUT channels
    //********************************************
    // add intra-group output channel
    for ( int dim = 0; dim < _n; ++dim ) { // always 1
      for ( int cnt = 0; cnt < (2*_p -1); ++cnt ) { // MS: local channel connects
        // MS: each router gets 3_p - 1 outputs 
        _output = ( 3 * _p - 1 ) * _n  * node + (2*_p-1) * dim  + cnt;

        _routers[node]->AddOutputChannel( _chan[_output], _chan_cred[_output] );

      #ifdef DRAGON_LATENCY
        _chan[_output]->SetLatency(10); // MS: local channels have short latency
        _chan_cred[_output]->SetLatency(10);
      #endif
      }
    }

    // add inter-group output channel

    for ( int cnt = 0; cnt < _p; ++cnt ) {
      //MS: Router offset + local offset + port offset
      _output = ( 3 * _p - 1 ) * node + ( 2 * _p - 1 ) + cnt;

      // _chan[_output].global = true;
      _routers[node]->AddOutputChannel( _chan[_output], _chan_cred[_output] );
      #ifdef DRAGON_LATENCY
      _chan[_output]->SetLatency(100);
      _chan_cred[_output]->SetLatency(100);
      #endif
    }


    //********************************************
    //   connect INPUT channels
    //********************************************
    // # of non-local nodes
    // MS: # of local + global connections
    // MS: Radix - nodes
    _num_ports_per_switch = (_k - _p);


    // intra-group GROUP channels
    // MS: we call this local connections
    for ( int dim = 0; dim < _n; ++dim ) {

      // MS: why do they have this line if it's overwritten by the next?
      _dim_ID = ((int) (node / ( powi(_p, dim)))); 

      // NODE ID within a group
      _dim_ID = node % _a;

      for ( int cnt = 0; cnt < (2*_p-1); ++cnt ) {

        if ( cnt < _dim_ID)  {

          _input =  
          // MS: bump to beginning of the group
          grp_ID  * _num_ports_per_switch * _a -
          // MS: bump to first local port in target switch
          (_dim_ID - cnt) *  _num_ports_per_switch +
          // 
          _dim_ID * _num_ports_per_switch + 
          (_dim_ID - 1);
        } else {

         _input =  grp_ID * _num_ports_per_switch * _a + 
         _dim_ID * _num_ports_per_switch + 
         (cnt - _dim_ID + 1) * _num_ports_per_switch + 
         _dim_ID;

        }

        if (_input < 0) {
          cout << " ERROR: _input less than zero " << endl;
          exit(-1);
        }

       _routers[node]->AddInputChannel( _chan[_input], _chan_cred[_input] );
     }
   }


   // add INPUT channels -- "optical" channels connecting the groups
   // MS: we call these global channels
   int total_groups = _g;
   int my_group = node /_a;
   int to_group;
   int to_port;
   int my_switch = node;
   int my_switch_local = my_switch % _a;

   for ( int cnt = 0; cnt < _p; ++cnt ) {
      // MS: get destination port based on the relative configuration
      to_group = ( my_switch_local * _p + cnt ) % total_groups;
      // the accepting port is based on how many forward hops it takes to get to the source group
      to_port = (to_group + _g - my_group) % _g; // forward distance
      int router_offset = to_port / _p;
      int port_offset = to_port % _p;
      _input = to_group * _num_ports_per_switch * _a + // group offset
                router_offset * _num_ports_per_switch + (2 * _p - 1) + port_offset;
      _routers[node]->AddInputChannel( _chan[_input], _chan_cred[_input] );
    }

  }

  cout<<"Done links"<<endl;
}


int DragonFlyRelative::GetN( ) const
{
  return _n;
}

int DragonFlyRelative::GetK( ) const
{
  return _k;
}

void DragonFlyRelative::InsertRandomFaults( const Configuration &config )
{
 
}

double DragonFlyRelative::Capacity( ) const
{
  return (double)_k / 8.0;
}

// routing functions are stored globally and usually added in ../routefunc.cpp
void DragonFlyRelative::RegisterRoutingFunctions(){

  gRoutingFunctionMap["min_dragonflyrelative"] = &min_dragonflyrelative;
  gRoutingFunctionMap["ugal_dragonflyrelative"] = &ugal_dragonflyrelative;
}

// route based on the shortest path algorithm
void min_dragonflyrelative( const Router *r, const Flit *f, int in_channel, 
		       OutputSet *outputs, bool inject )
{
  outputs->Clear( );

  if(inject) {  // create a new packet, add to random injection site
    int inject_vc= RandomInt(gNumVCs-1);
    outputs->AddRange(-1, inject_vc, inject_vc);
    return;
  }

  int _grp_num_routers= gAA;

  int dest  = f->dest;
  int rID =  r->GetID(); 

  int grp_ID = int(rID / _grp_num_routers); 
  int debug = f->watch;
  int out_port = -1;
  int out_vc = 0;
  int dest_grp_ID=-1;

  // came from a processor (speculatively):
  if ( in_channel < gPP ) {
    out_vc = 0;
    f->ph = 0;
    if (dest_grp_ID == grp_ID) { // does this ever get trigger?
      f->ph = 1;
    }
  }

  if(f->id == 0){
    out_port = dragonflyrelative_port(rID, f->src, dest, true);
  } else {
    out_port = dragonflyrelative_port(rID, f->src, dest, false);
  }

  

  // optical link
  if( out_port >= gPP + (gAA-1) ) {
    f->ph = 1;
  }

  out_vc = f->ph;
  if (debug)
    *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
	       << "	through output port : " << out_port 
	       << " out vc: " << out_vc << endl;
  // sets the output element to send flit via those specified virtual channels
  outputs->AddRange( out_port, out_vc, out_vc ); 
}


//Basic adaptive routing algorithm for the dragonfly
void ugal_dragonflyrelative( const Router *r, const Flit *f, int in_channel, 
			OutputSet *outputs, bool inject )
{
  //need 3 VCs for deadlock freedom

  assert(gNumVCs==3);
  outputs->Clear( );
  if(inject) {
    int inject_vc= RandomInt(gNumVCs-1);
    outputs->AddRange(-1, inject_vc, inject_vc);
    return;
  }
  
  //this constant biases the adaptive decision toward minimum routing
  //negative value would biases it towards nonminimum routing
  int adaptive_threshold = 30;

  int _grp_num_routers= gAA;
  int _grp_num_nodes =_grp_num_routers*gPP;
  int _network_size =  gAA * gPP * gGG;

 
  int dest  = f->dest;
  int rID =  r->GetID(); 
  int grp_ID = (int) (rID / _grp_num_routers);
  int dest_grp_ID = int(dest/_grp_num_nodes);

  int debug = f->watch;
  int out_port = -1;
  int out_vc = 0;
  int min_queue_size;
  int nonmin_queue_size;
  int intm_grp_ID;
  int intm_rID;

  if(debug){
    cout<<"At router "<<rID<<endl;
  }
  int min_router_output, nonmin_router_output;
  
  //at the source router, make the adaptive routing decision
  if ( in_channel < gPP )   {
    //dest are in the same group, only use minimum routing
    if (dest_grp_ID == grp_ID) {
      f->ph = 2;
    } else {
      //select a random node
      f->intm =RandomInt(_network_size - 1);
      intm_grp_ID = (int)(f->intm/_grp_num_nodes);
      if (debug){
	cout<<"Intermediate node "<<f->intm<<" grp id "<<intm_grp_ID<<endl;
      }
      
      //random intermediate are in the same group, use minimum routing
      if(grp_ID == intm_grp_ID){
	f->ph = 1;
      } else {
	//congestion metrics using queue length, obtained by GetUsedCredit()
	min_router_output = dragonflyrelative_port(rID, f->src, f->dest); 
      	min_queue_size = max(r->GetUsedCredit(min_router_output), 0) ; 

      
	nonmin_router_output = dragonflyrelative_port(rID, f->src, f->intm);
	nonmin_queue_size = max(r->GetUsedCredit(nonmin_router_output), 0);

	//congestion comparison, could use hopcnt instead of 1 and 2
	if ((1 * min_queue_size ) <= (2 * nonmin_queue_size)+adaptive_threshold ) {	  
	  if (debug)  cout << " MINIMAL routing " << endl;
	  f->ph = 1;
	} else {
	  f->ph = 0;
	}
      }
    }
  }

  //transition from nonminimal phase to minimal
  if(f->ph==0){
    intm_rID= (int)(f->intm/gPP);
    if( rID == intm_rID){
      f->ph = 1;
    }
  }

  //port assignement based on the phase
  if(f->ph == 0){
    out_port = dragonflyrelative_port(rID, f->src, f->intm);
  } else if(f->ph == 1){
    out_port = dragonflyrelative_port(rID, f->src, f->dest);
  } else if(f->ph == 2){
    out_port = dragonflyrelative_port(rID, f->src, f->dest);
  } else {
    assert(false);
  }

  //optical dateline
  if (f->ph == 1 && out_port >=gPP + (gAA-1)) {
    f->ph = 2;
  }  

  //vc assignemnt based on phase
  out_vc = f->ph;

  outputs->AddRange( out_port, out_vc, out_vc );
}
