#ifndef JSK_FOOTSTEP_PLANNER_GRID_STATE_PERCEPTION_H_
#define JSK_FOOTSTEP_PLANNER_GRID_STATE_PERCEPTION_H_

#include "jsk_footstep_planner/grid_state.h"

namespace jsk_footstep_planner {
  class PerceptionGridMap : public GridMap <CostedGridState>
  {
  public:
    typedef boost::shared_ptr< PerceptionGridMap > Ptr;
    typedef boost::shared_ptr< CostedGridState > StatePtr;
    typedef boost::function<bool(GridState::Ptr)> UpdateCostFunction;

    PerceptionGridMap(int _x, int _y) : GridMap(_x, _y)
    {
    }

    inline virtual GridState::Ptr getState(int ix, int iy)
    {
      GridState::Ptr pt = state_list_.at(index(ix, iy));
      if(!pt) {
        GridState::Ptr new_pt(new CostedGridState(ix, iy));
        updateCost(new_pt);
        return new_pt;
      }
      return pt;
    }

    virtual bool updateCost(GridState::Ptr st)
    {
      return cost_func_(st);
    }

    virtual void setCostFunction(UpdateCostFunction func)
    {
      cost_func_ = func;
    }
  protected:
    virtual void createGrid() {
      state_list_.clear();
      state_list_.resize(size_x_ * size_y_);
    }
    using GridMap::size_x_;
    using GridMap::size_y_;
    using GridMap::state_list_;

    UpdateCostFunction cost_func_;
  };
}

#endif
