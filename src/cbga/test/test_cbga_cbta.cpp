// standard libaray
#include <stdio.h>
#include <vector>
#include <ctime>
#include <tuple>
#include <algorithm>
#include <bitset>
// opencv
#include "opencv2/opencv.hpp"

// self-defined library
#include "graph/graph.hpp"
#include "graph/algorithms/astar.hpp"
#include "map/square_grid.hpp"
#include "vis/graph_vis.hpp"

#include "ltl/buchi_automaton.hpp"
#include "ltl/product_automaton.hpp"

#include "cbga/cbga_agent.hpp"
#include "cbga/cbga_task.hpp"
#include "cbba/cbba_task.hpp"

#include "cbta/graph_lift.hpp"
#include "cbta/hcost_interface.hpp"


using namespace cv;
using namespace librav;

int main(int argc, char** argv){
   	/*** 0. Preprocessing CBTA data ***/
	std::vector<double> radius_ = {4, 3, 3, 2};
	std::map<int, TileTraversalData> tile_travelsal_data;
	int idx = 0;
	for(auto &r_min: radius_){
		tile_travelsal_data[idx] = HCost::hcost_preprocessing(r_min);
		idx++;
	}

    /************************************************************************************************************/
	/*********************************          Initialize: Map         *****************************************/
	/************************************************************************************************************/
	/*** 1. Create a empty square grid ***/
	std::shared_ptr<SquareGrid> grid = GraphFromGrid::CreateSquareGrid();
	
	/*** 2. Define the target regions ***/
	// Define LTL specification
	LTLFormula Global_LTL;
	for (int i = 0; i < Global_LTL.task_info.size();i++)
		grid->SetInterestedRegionLabel(Global_LTL.task_info[i].pos_,Global_LTL.task_info[i].idx_);	
	// Decompose the global LTL_expression
	LTLDecomposition::GlobalLTLDecomposition(Global_LTL);

    CBGATasks tasks(Global_LTL);
    tasks.GetAllTasks();

	/*** 3. Construct a graph from the square grid ***/
	std::shared_ptr<Graph_t<SquareCell *>> grid_graph = GraphFromGrid::BuildGraphFromSquareGrid(grid, false, true);

	/*** 4. Initialize the info of agents ***/
	std::vector<cbga_Agent> agents =  CBGA::InitializeAgents();


    /************************************************************************************************************/
	/*********************************          Pre-Porcess for CBTA         ************************************/
	/************************************************************************************************************/
	int historyH = 4;
	//std::shared_ptr<Graph_t<LiftedSquareCell>> lifted_graph = GraphLifter::CreateLiftedGraph(HistoryH, graph);
    std::cout << "Start lifted graph " << std::endl;
    std::shared_ptr<Graph_t<LiftedSquareCell *>> lifted_graph = GraphLifter::BuildLiftedGraph(historyH, grid_graph);
	std::cout << "End lifted graph" << std::endl;


    /************************************************************************************************************/
	/******************************************          CBGA         *******************************************/
	/************************************************************************************************************/
	/*** 5. Start CBGA ***/
	bool flag = false;
	while(flag == false){
		CBGA::bundle_construction(tasks, Global_LTL, grid_graph, lifted_graph, tile_travelsal_data, grid, agents);
		
		// Increase the iteration
		for (int i = 0; i < agents[0].num_agents_; i++){
			agents[i].iter_++;
			agents[i].iteration_neighbors_[agents[i].idx_] = agents[i].iter_;
			agents[i].history_.iter_neighbors_his.push_back(agents[i].iteration_neighbors_);
		}
		
		CBGA::consensus(agents, tasks);
		CBGA::update_iteration_number(agents);
		CBGA::bundle_remove(agents);
		flag = CBGA::success_checker(agents, tasks);
		std::cout << "Iteration is " << agents[0].iter_ << std::endl;

	}

	std::cout << "Final Result " << std::endl;
	for (auto &agent:agents){
		std::cout << "For agent " << agent.idx_ << std::endl;
		std::cout << "Path is " << std::endl;
		for (auto &e: agent.cbga_path_){
			std::cout << e << " ";
		}
		std::cout << std::endl;
		// std::cout << "winning bid matrix is " << std::endl;
		// std::cout << agent.winning_bids_matrix_ << std::endl;
		// std::cout << "assignment matrix is " << std::endl;
		// std::cout << agent.assignment_matrix_ << std::endl;
	}


	int num_agents = 4;
	Path_t<SquareCell*> path_origin[num_agents];
	std::vector<std::string> buchi_regions;
	buchi_regions.push_back("p0");
	buchi_regions.push_back("p1");
	buchi_regions.push_back("p2");
	buchi_regions.push_back("p3");
	buchi_regions.push_back("p4");
	buchi_regions.push_back("p5");
	buchi_regions.push_back("p6");
	buchi_regions.push_back("p7");
	buchi_regions.push_back("p8");

	for (auto it_ag = agents.begin(); it_ag != agents.end(); it_ag++){
		
		/*** 8.1 Rebuild the local LTL specification based on current bundle/path ***/
		std::string ltl_formula = tasks.local_formula_recreator((*it_ag).cbga_path_);
		std::cout << "The specification is " << ltl_formula << std::endl;
		// std::vector<std::vector<std::string>> buchi_regions = LTLDecomposition::ObtainBuchiRegion({ltl_formula});

		/*** 8.2 Generate the corresponding buchi regions ***/
        std::shared_ptr<Graph_t<BuchiState *, std::vector<int64_t>>> buchi_graph = BuchiAutomaton::BuildBuchiGraph(ltl_formula,buchi_regions);
        std::vector<int64_t> buchi_acc = (*buchi_graph->FindVertex(0)).state_->acc_state_idx;
		//std::shared_ptr<Graph_t<BuchiState>> buchi_graph = BuchiAutomaton::CreateBuchiGraph(ltl_formula,buchi_regions.front());

		/*** 8.4 Construct a product graph ***/
        std::shared_ptr<Graph_t<ProductState *>> product_graph = std::make_shared<Graph_t<ProductState *>>();
        int64_t start_id_grid = (*it_ag).init_pos_;
        int64_t virtual_start_state_id = ProductAutomaton::SetVirtualStartState(product_graph, buchi_graph, grid_graph, start_id_grid);
		//std::shared_ptr<Graph_t<ProductState>> product_graph_new = std::make_shared<Graph_t<ProductState>>();

        GetProductNeighbor product_neighbor(grid_graph, buchi_graph);
        auto path = AStar::ProductIncSearch(product_graph, virtual_start_state_id, buchi_acc, GetNeighbourFunc_t<ProductState*, double>(product_neighbor));
        
		if (!(*it_ag).cbga_path_.empty()){
        	for(auto &e: path){
            	path_origin[(*it_ag).idx_].push_back(e->grid_vertex_->state_);
       	 	}
			std::cout << "The path length for vehicle "  << (*it_ag).idx_ << " is " << path_origin[(*it_ag).idx_].size() << std::endl;
		}
		else{
			path_origin[(*it_ag).idx_].push_back(path[0]->grid_vertex_->state_);
		}
		
	}



	/*** 9.Visualize the map and graph ***/
	// Image Layouts: square grid -> graph -> path
	GraphVis vis;
	Mat vis_img;
	vis.VisSquareGrid(*grid, vis_img);
	vis.VisSquareGridGraph(*grid_graph, vis_img, vis_img, false);
	// // put the path on top of the graph
	vis.VisSquareGridPath(path_origin[0], vis_img, vis_img, 0);
	vis.VisSquareGridPath(path_origin[1], vis_img, vis_img, 1);
	vis.VisSquareGridPath(path_origin[2], vis_img, vis_img, 2);
	vis.VisSquareGridPath(path_origin[3], vis_img, vis_img, 3);

	// display visualization result
	namedWindow("Processed Image", WINDOW_NORMAL ); // WINDOW_AUTOSIZE
	imshow("Processed Image", vis_img);
	imwrite("result_cbga_cbta.jpg",vis_img);

	return 0;	
}