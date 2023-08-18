#include "relaxed_task_graph.h"

#include <iostream>
#include <vector>
#include <queue>
#include <limits> 

using namespace std;


namespace planopt_heuristics {
RelaxedTaskGraph::RelaxedTaskGraph(const TaskProxy &task_proxy)
    : relaxed_task(task_proxy),
      variable_node_ids(relaxed_task.propositions.size()) {
    /*
      TODO: add your code for exercise 2 (b) here. Afterwards
        - variable_node_ids[i] should contain the node id of the variable node for variable i
        - initial_node_id should contain the node id of the initial node
        - goal_node_id should contain the node id of the goal node
        - the graph should contain precondition and effect nodes for all operators
        - the graph should contain all necessary edges.
    */

    for(int i = 0; i < (int) relaxed_task.propositions.size(); i++){
        //relaxed_task.propositions[i] == variable_node_ids[i] == graph.nodes[i]
        variable_node_ids[i] = relaxed_task.propositions[i].id;    
        graph.add_node(NodeType::OR, 0);
    }

    initial_node_id = graph.add_node(NodeType::AND, 0);
    for (PropositionID p_id : relaxed_task.initial_state){
        graph.add_edge(p_id, initial_node_id);
    }

    goal_node_id = graph.add_node(NodeType::AND, 0);
    for (PropositionID p_id : relaxed_task.goal){
        graph.add_edge(goal_node_id, p_id);
    }

    for (RelaxedOperator op : relaxed_task.operators){

        NodeID effect_node = graph.add_node(NodeType::AND, op.cost);
        
        NodeID formula_node = graph.add_node(NodeType::AND, 0);
        for (PropositionID p_id : op.preconditions){
            graph.add_edge(formula_node, p_id);
        }
        graph.add_edge(effect_node, formula_node);

        //For every conditional effect in the operator, there is an arc from variable node n_v to the effect node
        for (PropositionID p_id : op.effects){
            graph.add_edge(p_id, effect_node);
        }      
    }
}

void RelaxedTaskGraph::change_initial_state(const GlobalState &global_state) {
    // Remove all initial edges that where introduced for relaxed_task.initial_state.
    for (PropositionID id : relaxed_task.initial_state) {
        graph.remove_edge(variable_node_ids[id], initial_node_id);
    }

    // Switch initial state of relaxed_task
    relaxed_task.initial_state = relaxed_task.translate_state(global_state);

    // Add all initial edges for relaxed_task.initial_state.
    for (PropositionID id : relaxed_task.initial_state) {
        graph.add_edge(variable_node_ids[id], initial_node_id);
    }
}

bool RelaxedTaskGraph::is_goal_relaxed_reachable() {
    // Compute the most conservative valuation of the graph and use it to
    // return true iff the goal is reachable in the relaxed task.

    graph.most_conservative_valuation();
    return graph.get_node(goal_node_id).forced_true;
}

int RelaxedTaskGraph::additive_cost_of_goal() {
    // Compute the weighted most conservative valuation of the graph and use it
    // to return the h^add value of the goal node.

    // TODO: add your code for exercise 2 (c) here.
    graph.weighted_most_conservative_valuation();
    return graph.get_node(goal_node_id).additive_cost;
}

int RelaxedTaskGraph::ff_cost_of_goal() {
    // TODO: add your code for exercise 2 (e) here.
    graph.weighted_most_conservative_valuation();

    unordered_set<NodeID> already_expanded;
    queue<NodeID> queue;
    int sum_cost_effects = 0;

    queue.push(goal_node_id);

    while(!queue.empty()){
        AndOrGraphNode current_node = graph.get_node(queue.front());
        queue.pop();

        
        if(already_expanded.find(current_node.id) != already_expanded.end()){
            continue;
        }

        already_expanded.insert(current_node.id);
        
        if(current_node.type == NodeType::AND){

            //Every successor will be expanded
            for(NodeID successor_id : current_node.successor_ids){
                AndOrGraphNode successor_node = graph.get_node(successor_id);

                if(already_expanded.find(successor_id) == already_expanded.end()){
                    queue.push(successor_id);  
                }   
            }
        }
        else if(current_node.type == NodeType::OR){
            //Only achiever will be expanded
            if(already_expanded.find(current_node.achiever) == already_expanded.end()){
                queue.push(current_node.achiever);
            }
        }
    }

    for(NodeID node_id : already_expanded){
        sum_cost_effects += graph.get_node(node_id).direct_cost;
    }

    return sum_cost_effects;

}
}
