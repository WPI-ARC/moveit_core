/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \Author: Benjamin Cohen /bcohen@willowgarage.com, E. Gil Jones **/

#ifndef _ENVIRONMENT_CHAIN3D_TYPES_H_
#define _ENVIRONMENT_CHAIN3D_TYPES_H_

#include <vector>

namespace sbpl_interface {

static unsigned int HASH_TABLE_SIZE = 32*1024;

static inline unsigned int intHash(unsigned int key)
{
  key += (key << 12); 
  key ^= (key >> 22);
  key += (key << 4);
  key ^= (key >> 9);
  key += (key << 10);
  key ^= (key >> 2);
  key += (key << 7);
  key ^= (key >> 12);
  return key;
}
struct EnvChain3DHashEntry
{
  unsigned char dist; // distance to closest obstacle
  int stateID; // hash entry ID number
  int action; // which action in the list was required to get here
  int xyz[3]; // tip link coordinates in space
  std::vector<int> coord; // position in the angle discretization
  std::vector<double> angles; // position of joints in continuous space
};

/** @brief struct that describes a basic joint constraint */
struct EnvChain3DGoalPose
{
  bool is_6dof_goal;
  int type;
  int xyz_disc_tolerance;
  int rpy_disc_tolerance;
  int xyz_disc[3];
  int rpy_disc[3];
  double xyz[3];
  double rpy[3];
  double q[4];
  double fangle;
  double xyz_tolerance[3];
  double rpy_tolerance[3];
};

/** main structure that stores environment data used in planning */
struct EnvChain3DPlanningData
{
  
  EnvChain3DPlanningData(std::vector<int*>& state_ID_to_index_mapping) : 
    state_ID_to_index_mapping_(state_ID_to_index_mapping),
    goal_hash_entry_(NULL),
    start_hash_entry_(NULL),
    hash_table_size_(HASH_TABLE_SIZE)
  {
    coord_to_state_ID_table_.resize(hash_table_size_);
  }

  ~EnvChain3DPlanningData() {
    for(unsigned int i = 0; i < coord_to_state_ID_table_.size(); i++) {
      delete state_ID_to_coord_table_[i];
    }
  }

  unsigned int getHashBin(const std::vector<int>& coord) {
    unsigned int val = 0;
    
    for(size_t i = 0; i < coord.size(); i++)
      val += intHash(coord[i]) << i;
    
    return intHash(val) & (hash_table_size_-1);
  }

  EnvChain3DHashEntry* addHashEntry(const std::vector<int>& coord,
                                    const std::vector<double>& angles,
                                    const int(&xyz)[3],
                                    int action) 
  {
    EnvChain3DHashEntry* new_hash_entry = new EnvChain3DHashEntry();
    new_hash_entry->stateID = state_ID_to_coord_table_.size();
    new_hash_entry->coord = coord;
    new_hash_entry->angles = angles;
    memcpy(new_hash_entry->xyz, xyz, sizeof(int)*3);
    new_hash_entry->action = action;
    state_ID_to_coord_table_.push_back(new_hash_entry);
    unsigned int bin = getHashBin(coord);
    coord_to_state_ID_table_[bin].push_back(new_hash_entry);

    //have to do for DiscreteSpaceInformation
    //insert into and initialize the mappings
    int* entry = new int [NUMOFINDICES_STATEID2IND];
    memset(entry, -1, NUMOFINDICES_STATEID2IND*sizeof(int));
    state_ID_to_index_mapping_.push_back(entry);
    if(new_hash_entry->stateID != (int)state_ID_to_index_mapping_.size()) {
      ROS_ERROR_STREAM("Size mismatch between state mappings");
    }
    return new_hash_entry;
  }

  EnvChain3DHashEntry* getHashEntry(const std::vector<int> &coord,
                                    int action) 
  {
    unsigned int bin = getHashBin(coord);
    for(unsigned int i = 0; i < coord_to_state_ID_table_[bin].size(); i++) {
      if(coord_to_state_ID_table_[bin][i]->coord == coord) {
        return coord_to_state_ID_table_[bin][i];
      }
    }
    return NULL;
  }

  bool convertFromStateIDsToAngles(const std::vector<int>& state_ids,
                                   std::vector<std::vector<double> >& angle_vector) const {
    angle_vector.resize(state_ids.size());
    for(unsigned int i = 0; i < state_ids.size(); i++) {
      if(state_ids[i] > (int) state_ID_to_coord_table_.size()-1) {
        return false;
      }
      angle_vector[i] = state_ID_to_coord_table_[i]->angles;
    }
    return true;
  }

  //internal data from DiscreteSpaceInformation
  std::vector<int*>& state_ID_to_index_mapping_;

  EnvChain3DHashEntry* goal_hash_entry_;
  EnvChain3DHashEntry* start_hash_entry_;

  unsigned int hash_table_size_;
  //maps from coords to stateID
  std::vector<std::vector<EnvChain3DHashEntry*> > coord_to_state_ID_table_;

  //vector that maps from stateID to coords	
  std::vector<EnvChain3DHashEntry*> state_ID_to_coord_table_;

};

class JointMotionPrimitive {
public:
  virtual void generateSuccessorState(const std::vector<double>& start,
                                      std::vector<double>& end) = 0;
};

class SingleJointMotionPrimitive : public JointMotionPrimitive {
public:
  SingleJointMotionPrimitive(unsigned int ind,
                             double delta) :
    ind_(ind),
    delta_(delta)
  {
  }
  
  virtual void generateSuccessorState(const std::vector<double>& start,
                                      std::vector<double>& end) {
    end = start;
    end[ind_] += delta_;
  }
protected:
  unsigned int ind_;
  double delta_;
};

}

#endif