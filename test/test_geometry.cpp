#include <gtest/gtest.h>
#include <VMat/geometry.h>
TEST(test_geometry, grid){
    using namespace vm;
    Point3i maxPoint{256,256,256};
    Point3i orig{0,0,0};
    Bound3i bound{orig,maxPoint};
    auto grid = bound.GenGrid(Vec3i{4,4,4}); // each cell is {64,64,64}

    auto iter = grid.BeginIter(Ray{{1,1,1},{0,0,0}});
    while(iter.Valid()){
        std::cout<<iter.tMin<<" "<<iter.tMax<<" "<<iter.CellIndex<<std::endl;
        ++iter;
    }
}
