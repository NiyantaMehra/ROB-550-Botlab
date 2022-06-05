#include <planning/astar.hpp>
using namespace std;

robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    cell_t goalCell = global_position_to_grid_cell(Point<float>(goal.x, goal.y), distances);
    Node* goalNode = new Node(goalCell.x, goalCell.y);
    goalNode->g_cost = 1.0e16;
    goalNode->h_cost = 0.0;
    goalNode->parent = NULL;

    cell_t startCell = global_position_to_grid_cell(Point<float>(start.x, start.y), distances);
    Node* startNode = new Node(startCell.x, startCell.y);
    startNode->g_cost = 0.0;
    startNode->h_cost = h_cost(startNode, goalNode);
    startNode->parent = NULL;

    PriorityQueue openList;
    openList.push(startNode);
    std::vector<Node*> closedList;
    //std::vector<Node*> searchedList;
    //searchedList.push_back(startNode);
    bool found_path = false;
    Node* nextNode;
    
    while (not openList.empty()) {
        nextNode = openList.pop();
        closedList.push_back(nextNode);
        cout<<nextNode->cell.x<<endl;
        cout<<nextNode->cell.y<<endl;
        if (found_path) {
            break;
        }

        expand_node(nextNode, distances, params, openList, closedList);

        //std::vector<Node*> children = expand_node(nextNode, distance, params, openList, closedList);

        // if (auto child : children) {

        //     if (is_in_list(child, closed)) {
        //         continue;
        //     }

        //     if (not openList.is_member(child)) {
        //         child->g_cost = g_cost(nextNode, child);
        //         child->h_cost = h_cost(child, goalNode);
        //         child->parent = nextNode;
        //         openList.push(child);
        //     }
        // }
         
cout<<"check 10"<<endl;
   
        if (nextNode->cell.x == goalNode->cell.x && nextNode->cell.y == goalNode->cell.y ) {
            std::cout << "Found Path!" << std::endl;
            goalNode = nextNode;
            found_path = true;
        }

    }

    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    path.path_length = path.path.size();

    if (found_path) {
        extract_node_path(goalNode, startNode, path, distances, path.utime);
    }

    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list) {
    cout<<"check 6"<<endl;
    for (auto item : list) {
        cout<<"check 7"<<endl;
        cout<<node->cell<<endl;
        if (node->cell.x == item->cell.x && node->cell.y == item->cell.y) {
            cout<<"check 8"<<endl;
            return true;
        }
    }
    return false;
}

bool is_in_map(Node* node, const ObstacleDistanceGrid& grid) {
    int x = node->cell.x;
    int y = node->cell.y;
    return grid.isCellInGrid(x, y);
}

bool is_obstacle(Node* node, const ObstacleDistanceGrid& grid, double minDistanceToObstacle) {

    if(grid(node->cell.x, node->cell.y) == -1) {
        return false;
    }

    return grid(node->cell.x, node->cell.y)  <= (float)minDistanceToObstacle;
}


double h_cost(Node* from, Node* goal) {
    int x = std::abs(goal->cell.x - from->cell.x);
    int y = std::abs(goal->cell.y - from->cell.y);
    return (x + y);
}
    // grid cost
    //double cost = 0;
//     if ( x >= y ) {
//         cost = 14 * y + 10 * (x - y);
//     }
//     else {
//         cost = 14 * x + 10 * (x -y);
//     }
//     return cost;
// }

double g_cost(Node* from, Node* to) {
    return from->g_cost + 1;
    // int x = std::abs(to->cell.x - from->cell.x);
    // int y = std::abs(to->cell.y - from->cell.y);

    // if (x==1 & y==1) {
    //     return from->g_cost + 14;
    // }
    // else {
    //     return from->g_cost + 10;
    // }
}


void expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params, PriorityQueue& openList, std::vector<Node*> closedList){
    
    const int xDeltas[4] = {-1, 0, 1, 0};
    const int yDeltas[4] = {0, 1, 0, -1};

    for(int i=0; i<4; i++){
        
        Node* neighbor = new Node(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]);
        
        cout<<"check 1"<<endl;
        if (openList.is_member(neighbor)) {
            neighbor = openList.get_member(neighbor);
        //     cout<<"check 1.11"<<endl;
        //     for(auto search_node : openList){
        //         cout<<"check 1.1"<<endl;
        //         if(neighbor->cell.x == search_node->cell.x && neighbor->cell.y == search_node->cell.y) {
        //             neighbor = search_node;
        //             cout<<"check 1.2"<<endl;
        //         }
        //     }
        // cout<<"check 2"<<endl;
        }
        
        //else {
        //    neighbor = new Node(neighbor->cell.x, neighbor->cell.y);
        //}
cout<<"check 3"<<endl;
        if (not is_in_list(neighbor, closedList) && is_in_map(neighbor, distances) && not is_obstacle(neighbor, distances, params.minDistanceToObstacle)) {
            if (not openList.is_member(neighbor)) {
                neighbor->g_cost = g_cost(node, neighbor);
                neighbor->h_cost = h_cost(node, neighbor);
                neighbor->parent = node;
                openList.push(neighbor);
                cout<<"check 4"<<endl;
                //searchedList.push_back(neighbor);
            }
            else if (neighbor->g_cost > g_cost(node, neighbor)) {
                neighbor->g_cost = g_cost(node, neighbor);
                neighbor->parent =  node;
                openList.push(neighbor);
                //searchedList.push_back(neighbor);
                cout<<"check 5"<<endl;
            }
        }
    }
    
}


void extract_node_path(Node * node, Node* startNode, robot_path_t& path, const ObstacleDistanceGrid& distances, int64_t utime){
    Node* current_node = node;
    while (current_node != startNode) {

        Point<float> curr;
        curr.x = current_node->cell.x;
        curr.y = current_node->cell.y;
        auto curr_trans = grid_position_to_global_position(curr, distances);

        pose_xyt_t pose;
        pose.utime = utime;
        pose.x = curr_trans.x;
        pose.y = curr_trans.y;
        pose.theta = 0.0;
        path.path.push_back(pose);
        current_node = current_node->parent;
    }

    std::reverse(path.path.begin(), path.path.end());
}


// std::vector<pose_xyt_t> extract_pose_path(std::vector<Node*> nodePath, const ObstacleDistanceGrid& distances){
// }
