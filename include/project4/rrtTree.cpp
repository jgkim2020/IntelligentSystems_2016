#include "rrtTree.h"

rrtTree::rrtTree(){
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
}

rrtTree::~rrtTree(){
    for (int i = 1; i < count; i++) {
        delete ptrTable[i];
    }
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}


// TODO
// 1. Copy your implementation of member functions in Project Assignment #2
// 2. Implement generateRRTst
void rrtTree::addVertex(point x_new, point x_rand, int idx_near) {
    // TODO
    if(root == NULL){
        count = 1;
        node *root = new node;
        ptrTable[0] = root;
        root->idx = 0;
        root->idx_parent = NULL;
        root->location = x_init;
        root->rand = x_init;
    }
    else{
        node *temp = new node;
        ptrTable[count] = temp;
        temp->idx = count;
        temp->idx_parent = idx_near;
        temp->rand = x_rand;
        temp->location = x_new;
        count++;
    }
}


int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    // TODO
    for(int i = 0; i < K; i++) {
        point random_State;
        if(i%5 == 0) random_State = x_goal;
        else random_State = randomState(x_max, x_min, y_max, y_min);
        int near_idx = nearestNeighbor(random_State);
        point new_State = newState(near_idx, random_State, MaxStep);
        if(!isCollision(new_State, ptrTable[near_idx]->location)) {
            addVertex(new_State, random_State, near_idx);
            if (new_State.x == x_goal.x && new_State.y == x_goal.y) return 0;
        }
    }
    return 1;
}


point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    // TODO
    point randompoint;
    randompoint.x = (rand() / (static_cast<double>(RAND_MAX)+1.0))*(x_max-x_min)+x_min;
    randompoint.y = (rand() / (static_cast<double>(RAND_MAX)+1.0))*(y_max-y_min)+y_min;
    return randompoint;
}


point rrtTree::newState(int idx_near, point x_rand, double MaxStep) {
    // TODO
    point newpoint;
    point nearpoint = ptrTable[idx_near]->location;
    double x = x_rand.x - nearpoint.x;
    double y = x_rand.y - nearpoint.y;
    double length = sqrt(x*x + y*y);
    if(x_rand.x == x_goal.x && x_rand.y == x_goal.y && MaxStep >= length) {
        newpoint.x = x_goal.x;
        newpoint.y = x_goal.y;
    }
    else {
        newpoint.x = nearpoint.x + x*MaxStep/length;
        newpoint.y = nearpoint.y + y*MaxStep/length;
    }
    return newpoint;
}


int rrtTree::nearestNeighbor(point x_rand) {
    // TODO
    double length;
    double minlength = 1000000000;
    int argmin = 0;
    for (int i=0; i<count; i++){
        length = (ptrTable[i]->location.x-x_rand.x)*(ptrTable[i]->location.x-x_rand.x)+(ptrTable[i]->location.y-x_rand.y)*(ptrTable[i]->location.y-x_rand.y);
        if (length < minlength){
            minlength = length;
            argmin = i;
        }
    }
    return argmin;
}


bool rrtTree::isCollision(point x1, point x2) {
    // TODO
    point temp;
    int tempx_idx, tempy_idx, pix_val;
    int range = 23, margin = 0; //VARIABLEJ VARIABLEK
    int x1_idx = (int) (x1.x / res + map_origin_x);
    int x2_idx = (int) (x2.x / res + map_origin_x);
    int y1_idx = (int) (x1.y / res + map_origin_y);
    int y2_idx = (int) (x2.y / res + map_origin_y);
    int check_num = 2 * std::max(std::abs(x1_idx - x2_idx), std::abs(y1_idx - y2_idx));

    for (int i = -margin; i <= check_num + margin; i++) {
        temp.x = x1.x + (x2.x - x1.x) * i / check_num;
        temp.y = x1.y + (x2.y - x1.y) * i / check_num;
        tempx_idx = (int) (temp.x / res + map_origin_x);
        tempy_idx = (int) (temp.y / res + map_origin_y);
        for (int j = -range; j < range + 1; j++) {
            for (int k = -range; k < range + 1; k++) {
                if (tempx_idx + j >= 0 && tempy_idx + k >= 0 && tempx_idx + j <= map.rows && tempy_idx + k <= map.cols) {
                    pix_val = map_original.at<uchar>(tempx_idx + j, tempy_idx + k);
                    if (pix_val == 0 || pix_val == 125) return true;
                }
            }
        }
    }
    return false;
}


std::vector<point> rrtTree::backtracking(){
    // TODO
    std::vector<point> path;
    int index = count - 1;
    while(index != 0){
        path.push_back(ptrTable[index]->location);
        index = ptrTable[index]->idx_parent;
    }
    path.push_back(ptrTable[0]->location);
    return path;
}


std::vector<int> rrtTree::Neighbors(point x_new, double radius) {
    // TODO implement Neighbors function
    std::vector<int> idx_neighbors;
    double length;
    for (int i = 0; i < count; i++){
        length = (ptrTable[i]->location.x-x_new.x)*(ptrTable[i]->location.x-x_new.x)+(ptrTable[i]->location.y-x_new.y)*(ptrTable[i]->location.y-x_new.y);
        if (length <= radius*radius) idx_neighbors.push_back(i);
    }
    return idx_neighbors;
}


double rrtTree::Cost(int idx_node) {
    // TODO implement Cost function
    if (idx_node == 0) return 0.0;
    else {
        point p1 = ptrTable[idx_node]->location;
        point p2 = ptrTable[ptrTable[idx_node]->idx_parent]->location;
        return Cost(ptrTable[idx_node]->idx_parent) + sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
    }
}


int rrtTree::optimalNeighbor(std::vector<int> idx_neighbors, int idx_near, point x_new) {
    // TODO implement optimalNeighbor function
    int idx_optimal = idx_near;
    point x_optimal = ptrTable[idx_near]->location;
    double cost_optimal = Cost(idx_near) + sqrt((x_optimal.x - x_new.x)*(x_optimal.x - x_new.x) + (x_optimal.y - x_new.y)*(x_optimal.y - x_new.y));
    double cost_temp;
    for (int i = 0; i < idx_neighbors.size(); i++) {
        x_optimal = ptrTable[idx_neighbors[i]]->location;
        cost_temp = Cost(idx_neighbors[i]) + sqrt((x_optimal.x - x_new.x)*(x_optimal.x - x_new.x) + (x_optimal.y - x_new.y)*(x_optimal.y - x_new.y));
        if (cost_temp < cost_optimal && !isCollision(x_optimal, x_new)) {
            cost_optimal = cost_temp;
            idx_optimal = idx_neighbors[i];
        }
    }
    return idx_optimal;
}


void rrtTree::rewire(int idx_new, std::vector<int> idx_neighbors, int idx_optimal) {
    // TODO implement rewire function
    double cost_temp;
    point x_neighbor;
    point x_new = ptrTable[idx_new]->location;
    for (int i = 0; i < idx_neighbors.size(); i++) {
        if (idx_optimal != idx_neighbors[i]) {
            x_neighbor = ptrTable[idx_neighbors[i]]->location;
            cost_temp = Cost(idx_new) + sqrt((x_neighbor.x - x_new.x)*(x_neighbor.x - x_new.x) + (x_neighbor.y - x_new.y)*(x_neighbor.y - x_new.y));
            if (cost_temp < Cost(idx_neighbors[i]) && !isCollision(x_neighbor, x_new)) {
                ptrTable[idx_neighbors[i]]->idx_parent = idx_new;
            }
        }
    }
}


int rrtTree::generateRRTst(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    // TODO 2.

    for(int i = 0; i < K; i++) {
        // generate random node
        point random_Node;
        if(i%5 == 2) random_Node = x_goal; //VARIABLEF
        else random_Node = randomState(x_max, x_min, y_max, y_min);
        // find nearest node
        int nearest_idx = nearestNeighbor(random_Node);
        // new node
        point new_Node = newState(nearest_idx, random_Node, MaxStep);
        if(!isCollision(new_Node, ptrTable[nearest_idx]->location)) {
            std::vector<int> neighbors_idx = Neighbors(new_Node, 1800); // find neighbors //VARIABLEI
            int optimal_idx = optimalNeighbor(neighbors_idx, nearest_idx, new_Node); // find optimal parent
            addVertex(new_Node, random_Node, optimal_idx); // connect along a optimal cost path
            rewire(count - 1, neighbors_idx, optimal_idx); // rewire
            if (new_Node.x == x_goal.x && new_Node.y == x_goal.y) return 0; // tree has reached goal (success)
        }
    }
    return 1; // tree could not reach goal (fail)
}
