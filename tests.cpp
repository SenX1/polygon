#include <gtest/gtest.h>
#include "alg.cpp" 

class GraphTest : public ::testing::Test {
protected:
    Graph graph;

    void SetUp() override {
        graph.load_from_file("spb_graph.txt");
    }
};

TEST_F(GraphTest, BFSTest) {
    double start_lat = 59.9470649, start_lon = 30.4141326;
    double goal_lat = 59.9469059, goal_lon = 30.4140936;

    Node* start = graph.find_closest_node(start_lat, start_lon);
    Node* goal = graph.find_closest_node(goal_lat, goal_lon);

    std::stringstream ss;
    std::streambuf* old = std::cout.rdbuf(ss.rdbuf());

    bfs(start, goal);

    std::cout.rdbuf(old);

    EXPECT_TRUE(ss.str().find("(59.9470649, 30.4141326) -> (59.9469986, 30.4140652) -> (59.9469059, 30.4140936) -> end") != std::string::npos);
}

TEST_F(GraphTest, DFSTest) {
    double start_lat = 59.9470649, start_lon = 30.4141326;
    double goal_lat = 59.9469059, goal_lon = 30.4140936;

    Node* start = graph.find_closest_node(start_lat, start_lon);
    Node* goal = graph.find_closest_node(goal_lat, goal_lon);

    std::stringstream ss;
    std::streambuf* old = std::cout.rdbuf(ss.rdbuf());

    dfs(start, goal);

    std::cout.rdbuf(old);

    EXPECT_TRUE(ss.str().find("(59.9470649, 30.4141326) -> (59.9469986, 30.4140652) -> (59.9469059, 30.4140936) -> end") != std::string::npos);
}

TEST_F(GraphTest, DijkstraTest) {
    double start_lat = 59.9470649, start_lon = 30.4141326;
    double goal_lat = 59.9469059, goal_lon = 30.4140936;

    Node* start = graph.find_closest_node(start_lat, start_lon);
    Node* goal = graph.find_closest_node(goal_lat, goal_lon);

    std::stringstream ss;
    std::streambuf* old = std::cout.rdbuf(ss.rdbuf());

    dijkstra(start, goal);

    std::cout.rdbuf(old);

    EXPECT_TRUE(ss.str().find("(59.9470649, 30.4141326) -> (59.9469986, 30.4140652) -> (59.9469059, 30.4140936) -> end") != std::string::npos);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}