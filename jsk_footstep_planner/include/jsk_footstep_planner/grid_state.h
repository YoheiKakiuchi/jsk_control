#ifndef JSK_FOOTSTEP_PLANNER_GRID_STATE_H_
#define JSK_FOOTSTEP_PLANNER_GRID_STATE_H_

namespace jsk_footstep_planner {
  class GridState
  {
  public:
    typedef boost::shared_ptr<GridState> Ptr;
    GridState(int x, int y) : index_x_(x), index_y_(y)
    {
    }
    inline virtual int indexX() { return index_x_; }
    inline virtual int indexY() { return index_y_; }

    inline virtual int getOccupancy() { return 0; }
    inline virtual bool setOccupancy(int occ_) { return false; }
    inline virtual float getCost() { return 0.0; }
    inline virtual bool setCost(float c_) { return false; }
    inline virtual bool isValid() { return true; }
    // for boost unordered_map
    bool operator==(const GridState& other) const
    {
      return ((index_x_ == other.index_x_) &&
              (index_y_ == other.index_y_));
    }
  protected:
    int index_x_;
    int index_y_;
  };

  class OccupancyGridState : public GridState
  {
  public:
    typedef boost::shared_ptr<OccupancyGridState> Ptr;

    OccupancyGridState(int x, int y) : GridState(x, y), occupancy_(0)
    {
    }

    inline virtual int getOccupancy() { return occupancy_; }
    inline virtual bool setOccupancy(int occ_)
    {
      occupancy_ = occ_;
      return true;
    }
    inline virtual bool isValid()
    {
      if(occupancy_ == 0) {
        return true;
      }
      return false;
    }
  protected:
    int occupancy_;
  };

  class CostedGridState : public OccupancyGridState
  {
  public:
    typedef boost::shared_ptr<CostedGridState> Ptr;

    CostedGridState(int x, int y) : OccupancyGridState(x, y), cost_(0.0)
    {
    }

    inline virtual float getCost()
    {
      return cost_;
    }
    inline virtual bool setCost(float c_)
    {
      cost_ = c_;
      return true;
    }
    inline virtual bool isValid() {
      // TODO update for using cost
      if(occupancy_ == 0) {
        return true;
      }
      return false;
    }
  protected:
    float cost_;
  };

  // for boost unordered_map
  inline size_t hash_value(const GridState::Ptr& s)
  {
    return (std::abs(s->indexX() + 32000) << 16) + std::abs(s->indexY() + 32000);
  }

  template <class GStateT>
  class GridMap
  {
  public:
    typedef boost::shared_ptr< GridMap > Ptr;
    typedef boost::shared_ptr< GStateT > StatePtr;

    GridMap(int _x, int _y) : size_x_(_x), size_y_(_y)
    {
      createGrid();
    }
    inline virtual GridState::Ptr getState(int ix, int iy)
    {
      return state_list_.at(index(ix, iy));
    }
    inline virtual bool setCost(std::vector<float> in)
    {
      size_t cnt = state_list_.size() <= in.size() ? state_list_.size() : in.size();
      for(int i = 0; i < cnt; i++) {
        state_list_[i]->setCost(in[i]);
      }
    }
    inline virtual bool setCost(int ix, int iy, float cost = 0.0)
    {
      return getState(ix, iy)->setCost(cost);
    }
    inline virtual bool setOccupancy(std::vector<int> in)
    {
      size_t cnt = state_list_.size() <= in.size() ? state_list_.size() : in.size();
      for(int i = 0; i < cnt; i++) {
        state_list_[i]->setOccupancy(in[i]);
      }
    }
    inline virtual bool setOccupancy(int ix, int iy, int occupancy = 0)
    {
      return getState(ix, iy)->setOccupancy(occupancy);
    }
    inline virtual int getOccupancy(int ix, int iy)
    {
      return getState(ix, iy)->getOccupancy();
    }
    inline virtual float getCost(int ix, int iy)
    {
      return getState(ix, iy)->getCost();
    }
    inline virtual bool isValid(int ix, int iy)
    {
      return getState(ix, iy)->isValid();
    }

    inline virtual int sizeX() { return size_x_; }
    inline virtual int sizeY() { return size_y_; }
    inline virtual int index(int ix, int iy) { return ((size_x_ * iy) + ix); }
    inline virtual int inRange(int ix, int iy) { return (ix >= 0 && ix < size_x_ && iy >= 0 && iy < size_y_ ); }
  protected:
    virtual void createGrid() {
      state_list_.resize(size_x_ * size_y_);
      for (int y = 0; y < size_y_; y++) {
        for (int x = 0; x < size_x_; x++) {
          GridState::Ptr st(new GStateT(x, y));
          state_list_[index(x, y)] = st;
        }
      }
    }
    int size_x_;
    int size_y_;
    std::vector<GridState::Ptr > state_list_;
  };
}
#endif
