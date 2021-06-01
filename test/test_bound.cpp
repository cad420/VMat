#include <gtest/gtest.h>
#include <VMat/geometry.h>
#include <VMat/transformation.h>
    using namespace vm;

void test_grid(RayIntervalIter & iter,const std::vector<Point3i> & res, const Ray & r){
    int i = 0;
    while(iter.Valid()){
        auto p = Point3i{iter.CellIndex.x,iter.CellIndex.y,iter.CellIndex.z};
        ASSERT_EQ(p.x,res[i].x);
        ASSERT_EQ(p.y,res[i].y);
        ASSERT_EQ(p.z,res[i].z);
        ++iter;
        i++;
    }
    ASSERT_EQ(i,res.size());
}


TEST(test_geometry, grid){
    const Point3i maxPoint{256,256,256};
    const Point3i minPoint{0,0,0};
    const Bound3i bound{minPoint,maxPoint};
    const auto diag = bound.Diagonal();
    const Vec3i grid{4,4,4};
    const Vec3f cell(diag.x/grid.x,diag.y/grid.y,diag.z/grid.z);
    auto g = bound.GenGrid(grid); // each cell is {64,64,64}
    ASSERT_EQ(g.GridDimension,grid);
    ASSERT_EQ(g.Cell,cell );


    auto ray = Ray{{1,1,1},{0,0,0}};
    auto iter = g.IntersectWith(ray);
    ASSERT_TRUE(iter.Valid());
    std::vector<Point3i> res2{{0,0,0},{1,1,1},{2,2,2},{3,3,3}};
    test_grid(iter,res2,ray);

    ray = Ray{{2,1,0},{-0.5,-0.5, 0.5}};
    iter = g.IntersectWith(ray);
    ASSERT_TRUE(iter.Valid());
    res2 = std::vector<Point3i>{{0, 0, 0},{1, 0, 0},{2,0,0},{2,1,0},{3,1,0}};
    test_grid(iter,res2,ray);

    ray = Ray{{2,1,1},{-0.5,-0.5, -0.5}};
    iter = g.IntersectWith(ray);
    ASSERT_TRUE(iter.Valid());
    res2 = std::vector<Point3i>{{0, 0, 0},{1, 0, 0},{2,0,0},{2,1,1},{3,1,1}};
    test_grid(iter,res2,ray);

    ray = Ray{{2,1,0},{0,0,0}};
    iter = g.IntersectWith(ray);
    ASSERT_TRUE(iter.Valid());
    res2 =std::vector<Point3i>{{0, 0, 0},{1, 0, 0},{2,1,0},{3,1,0}};
    test_grid(iter,res2,ray);


    ray = Ray{{1,0,0},{-0.5, 0.5, 0.5}};
    iter = g.IntersectWith(ray);
    ASSERT_TRUE(iter.Valid());
    res2= std::vector<Point3i>{{0,0,0},{1,0,0},{2,0,0},{3,0,0}};
    test_grid(iter,res2,ray);

    // no intersection 
    ray = Ray{{0,1,0},{-0.5, 0.5, 0.5}};
    iter = g.IntersectWith(ray);
    ASSERT_FALSE(iter.Valid());

    // ray in a bounding face
    ray = Ray{{0.5,0.0,0.0},{0, 0.5, 0.0}};
    iter = g.IntersectWith(ray);
    ASSERT_TRUE(iter.Valid());
    res2= std::vector<Point3i>{{0,0,0},{1,0,0},{2,0,0},{3,0,0}};
    test_grid(iter,res2,ray);

    // ray in a grid boundary
    ray = Ray{{1,0,0},{0, 63, 63}};
    iter = g.IntersectWith(ray);
    ASSERT_TRUE(iter.Valid());
    res2= std::vector<Point3i>{{0,0,0},{1,0,0},{2,0,0},{3,0,0}};
    test_grid(iter,res2,ray);

    ray = Ray{{1,0,0},{0, 64, 64}};
    iter = g.IntersectWith(ray);
    ASSERT_TRUE(iter.Valid());
    res2= std::vector<Point3i>{{0,1,1},{1,1,1},{2,1,1},{3,1,1}};
    test_grid(iter,res2,ray);
}

TEST(test_rayiterator, raycast){

    Vec3i blockCount{4,4,4};
    Vec3i blockSize(256,256,256);
    Vec3i boundSize = blockCount * blockSize;
	Bound3i bound( { 0, 0, 0 }, boundSize.ToPoint3() );
	auto grid = bound.GenGrid(blockCount);
    Vec2i screenSize{1024,768};
    auto aspect = 1.0 * screenSize.x / screenSize.y;
    auto fov = 60.f;
    auto step = 0.01;
    auto eye = Point3f{ -500, -500, -500 };
    auto center = Point3f{ 0, 0, 0 };
    auto up = Vec3f{ 0, 1, 0 };

    auto lookAt = LookAt(eye, center, up);
    auto inverseLookAt = lookAt.Inversed();
    auto persp = Perspective( fov, aspect, 0.01, 1000 );
    auto invPersp = persp.Inversed();

    auto screenToPerps =
      Translate( -1, 1, 0 ) * Scale( 2, -2, 1 ) *
      Scale( 1.0 / screenSize.x, 1.0 / screenSize.y, 1.0 );


    auto screenToWorld = inverseLookAt * invPersp * screenToPerps;
	for ( int y = 0; y < screenSize.y; y++ ) {
		for ( int x = 0; x < screenSize.x; x++ ) {
			auto pScreen = Point3f( x, y, 0 );
			auto pWorld = screenToWorld * pScreen;
			auto dir = pWorld - eye;
			auto r = Ray( dir, eye );
			auto iter = grid.IntersectWith( r );
            float tPrev = iter.Pos+step, tCur, tMax = iter.Max - step;
			auto cellIndex = iter.CellIndex;
            while ( iter.Valid() ) {
                ++iter;
                tCur = iter.Pos;
                while ( tPrev < tCur && tPrev < tMax ) {
                    auto globalPos = r( tPrev );
                    auto & globalSamplePos = globalPos;
                    auto innerOffset = ( globalSamplePos.ToVector3() - Vec3f( cellIndex.ToVector3() * blockSize ) ).ToPoint3();

                    ASSERT_GE(innerOffset.x, 0.0-step);
                    ASSERT_LT(innerOffset.x, blockSize.x);

                    ASSERT_GE(innerOffset.y, 0.0-step);
                    ASSERT_LT(innerOffset.y, blockSize.y);

                    ASSERT_GE(innerOffset.z, 0.0-step);
                    ASSERT_LT(innerOffset.z, blockSize.z);

                    tPrev += step;
                }
				cellIndex = iter.CellIndex;
                tPrev = tCur;
            }
		}
	}
}