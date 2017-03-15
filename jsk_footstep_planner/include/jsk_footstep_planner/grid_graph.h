#ifndef JSK_FOOTSTEP_PLANNER_GRID_GRAPH_H_
#define JSK_FOOTSTEP_PLANNER_GRID_GRAPH_H_

#include "jsk_footstep_planner/graph.h"
#include "jsk_footstep_planner/grid_state_perception.h"

namespace jsk_footstep_planner
{
  template <class GStateT>
  class GridGraph: public Graph<GridState>
  {
  public:
    typedef boost::shared_ptr<GridGraph> Ptr;
    typedef GridMap<GStateT> GridType;
    GridGraph(typename GridType::Ptr gr)
    {
      gridmap_ = gr;
    }

    StatePtr getState(int ix, int iy) {
      if(gridmap_->inRange(ix, iy)) {
        return gridmap_->getState(ix, iy);
      }
      StatePtr p;
      p.reset();
      return p;
    }

    virtual std::vector<StatePtr> successors(StatePtr target_state)
    {
      std::vector<StatePtr> ret;
      int x_offset = target_state->indexX();
      int y_offset = target_state->indexY();
      for (int x = -1; x < 2; x++) {
        for (int y = -1; y < 2; y++) {
          if (x != 0 || y != 0) {
            StatePtr st =
              getState(x + x_offset, y + y_offset);
            if (!!st) {
              if(st->isValid()) {
                ret.push_back(st);
              }
            }
          }
        }
      }
      return ret;
    }
    virtual bool isGoal(StatePtr state)
    {
      return (state == goal_state_);
    }

    virtual double pathCost(StatePtr from,
                            StatePtr to, double prev_cost)
    {
      double gx = from->indexX() - to->indexX();
      double gy = from->indexY() - to->indexY();
      return prev_cost + std::sqrt(gx * gx + gy * gy) + to->getCost();
    }

  protected:
    //using Graph<GridState>::start_state_;
    //using Graph<GridState>::goal_state_;
    typename GridType::Ptr gridmap_;
  private:
  };

  typedef GridMap<GridState> SimpleGridMap;
  typedef GridMap<OccupancyGridState> OccupancyGridMap;
  typedef GridMap<CostedGridState> CostedGridMap;

  typedef GridGraph<GridState> SimpleGridGraph;
  typedef GridGraph<OccupancyGridState> OccupancyGridGraph;
  typedef GridGraph<CostedGridState> CostedGridGraph;
  typedef GridGraph<CostedGridState> PerceptionGridGraph;
}

#endif
