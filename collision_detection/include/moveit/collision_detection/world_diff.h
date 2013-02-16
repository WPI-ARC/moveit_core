/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Sachin Chitta */

#ifndef MOVEIT_COLLISION_DETECTION_WORLD_DIFF_
#define MOVEIT_COLLISION_DETECTION_WORLD_DIFF_

#include <moveit/collision_detection/world.h>
#include <boost/weak_ptr.hpp>

namespace collision_detection
{

  /** \brief Maintain a diff list of changes that have happened to a World. */
  class WorldDiff
  {
  public:

    /** \brief Constructor */
    WorldDiff();

    /** \brief Constructor */
    WorldDiff(const WorldPtr& world);

    /** \brief copy constructor. */
    WorldDiff(WorldDiff &other);

    ~WorldDiff();


    /** \brief Set which world to record.  Erases all previously recorded
     * changes.  */
    void reset(const WorldPtr& world);

    /** \brief Turn off recording and erase all previously recorded changes. */
    void reset();

    /** \brief Return all the changes that have been recorded */
    const std::map<std::string, World::Action>& getChanges() const;

    typedef std::map<std::string, World::Action>::const_iterator Iterator;
    /** iterator pointing to first change */
    Iterator begin()
    {
      return changes_.begin();
    }
    /** iterator pointing to end of changes */
    Iterator end()
    {
      return changes_.end();
    }
    /** number of changes stored */
    std::map<std::string, World::Action>::size_type size()
    {
      return changes_.size();
    }
    /** find changes for a named object */
    Iterator find(const std::string& id)
    {
      return changes_.find(id);
    }

    /** \brief Clear the internally maintained vector of changes */
    void clearChanges();

  private:
    /** \brief Notification function */
    static void notify(WorldDiff*, const World::ObjectConstPtr&, World::Action);

    /** keep changes in a map so they can be coalesced */
    std::map<std::string, World::Action> changes_;

    /* used to unregister the notifier */
    boost::weak_ptr<World> world_;
  };

  typedef boost::shared_ptr<WorldDiff> WorldDiffPtr;
  typedef boost::shared_ptr<const WorldDiff> WorldDiffConstPtr;
}

#endif
