// C++ STL header
#include <stdio.h>
#include <vector>
#include <cstring>
#include <iostream>
#include <ctime>
#include <bitset>
#include <tuple>
#include <string>

// User-defined header
// opencv
#include "opencv2/opencv.hpp"

// user
#include "graph/algorithms/astar.hpp"
#include "vis/graph_vis.hpp"
#include "map/square_grid.hpp"
#include "graph/graph.hpp"
#include "ltl/buchi_automaton.hpp"
#include "ltl/product_automaton.hpp"

using namespace cv;
using namespace librav;
int main(int argc, char** argv )
{
    //======================================== SET GRID MAP =======================================//
	std::shared_ptr<SquareGrid> grid = GraphFromGrid::CreateSquareGrid(10,10,1);

	grid->SetInterestedRegionLabel(99,1);
    // grid->SetInterestedRegionLabel(99,2);

	grid->SetInterestedRegionLabel(9,2);
    grid->SetInterestedRegionLabel(42,4);
    grid->SetInterestedRegionLabel(13,13);
    // grid->SetInterestedRegionLabel(80,4);

    grid->SetObstacleRegionLabel(23,1);
    grid->SetObstacleRegionLabel(33,1);
    grid->SetObstacleRegionLabel(43,1);
    grid->SetObstacleRegionLabel(53,1);
    grid->SetObstacleRegionLabel(63,1);
    grid->SetObstacleRegionLabel(73,1);
    grid->SetObstacleRegionLabel(83,1);
    grid->SetObstacleRegionLabel(93,1);

	std::shared_ptr<Graph_t<SquareCell *>> grid_graph = GraphFromGrid::BuildGraphFromSquareGrid(grid,false, false);

    //======================================== SET BUCHI AUTOMATON =================================//
    // Buchi Automaton
    std::vector<std::string> buchi_regions;
	std::string ltl_formula = "[] p0 && [] !p1 && <> (p4 && ( p4 -> <> p13))";
    // std::string ltl_formula = "[] p0 && <>[] p1 && <> p2 && <> p3";
	//buchi_regions.push_back("p1");
	buchi_regions.push_back("p0");
	buchi_regions.push_back("p1");
	buchi_regions.push_back("p2");
	buchi_regions.push_back("p13");
    buchi_regions.push_back("p4");
    // buchi_regions.push_back("p5");
	std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = BuchiAutomaton::BuildBuchiGraph(ltl_formula,buchi_regions);
    std::vector<int64_t> buchi_acc = (*buchi_graph->FindVertex(0)).state_->acc_state_idx;
    
    
    //======================================== DEBUG - BUCHI =======================================//
    // std::cout << "The Acc State is " << std::endl;
    // for(auto &acc: buchi_acc)
    //     std::cout << acc << " ";
    // std::cout << std::endl;

    // int64_t buchi_init = (*buchi_graph->FindVertex(0)).state_->init_state_idx_;
    // std::cout << "The Init State is " << std::endl;
    // std::cout << buchi_init << std::endl;


    //======================================== SET PRODUCT GRAPH ===================================//
    // Product automaton 
    //std::shared_ptr<Graph_t<ProductState *>> product_graph = ProductAutomaton::BuildProductGraph(grid_graph, buchi_graph);
    std::shared_ptr<Graph_t<ProductState *>> product_graph = std::make_shared<Graph_t<ProductState *>>();
    int64_t start_id_grid = 0;
    int64_t virtual_start_state_id = ProductAutomaton::SetVirtualStartState(product_graph, buchi_graph, grid_graph, start_id_grid);
    //========================================= DEBUG - PRODUCT=====================================//
    // auto product_vertices = product_graph->GetAllVertex();
    // for(auto &e: product_vertices){
    //     std::cout << "Grid vertex is " << e->state_->grid_vertex_->state_->id_ << " Buchi vertex is " << e->state_->buchi_vertex_->state_->id_ << std::endl;
    //     std::cout << "Product id is " << e->state_->id_ << std::endl;
    // }

    //========================================= SEARCH =============================================//
    GetProductNeighbor product_neighbor(grid_graph, buchi_graph);
    auto path = AStar::ProductIncSearch(product_graph, virtual_start_state_id, buchi_acc, GetNeighbourFunc_t<ProductState*, double>(product_neighbor));
    Path_t<SquareCell *> path_vis;
    for(auto &e: path){
        path_vis.push_back(e->grid_vertex_->state_);
    }

    for (auto &ee:path_vis){
        std::cout << ee->id_ << " ";
    }
    std::cout << std::endl;


    //========================================= VISUALIZATION =============================================//
    /*** Visualize the map and graph ***/
	Mat vis_img;
	/*** Image Layouts: (map) -> square grid -> graph -> path ***/
	/*** you can visualize the squre grid by itself or overlay it on the map image ***/
	GraphVis::VisSquareGrid(*grid, vis_img);
	GraphVis::VisSquareGridGraph(*grid_graph, vis_img, vis_img, true);
	GraphVis::VisSquareGridPath(path_vis, vis_img, vis_img);

	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);

	waitKey(0);

    return 0;
}