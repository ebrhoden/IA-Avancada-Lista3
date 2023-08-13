#include "relaxed_task_graph.h"

#include <iostream>
#include <vector>

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

    for (Proposition p: relaxed_task.propositions){
        variable_node_ids.push_back(p.id);  
        graph.add_node(NodeType::OR);   
    }

    initial_node_id = graph.add_node(NodeType::AND);
    for (PropositionID p_id : relaxed_task.initial_state){
        graph.add_edge(p_id, initial_node_id);
    }

    goal_node_id = graph.add_node(NodeType::AND);
    for (PropositionID p_id : relaxed_task.goal){
        graph.add_edge(goal_node_id, p_id);
    }

    for (RelaxedOperator op : relaxed_task.operators){

        NodeID operator_node = graph.add_node(NodeType::AND);
        

        /*
        NodeID precondition_node = graph.add_node(NodeType::AND);
        for (PropositionID p_id : op.preconditions){
            graph.add_edge(precondition_node, p_id);
        }
        graph.add_edge(operator_node, precondition_node);
        
        NodeID effect_node = graph.add_node(NodeType::AND);
        for (PropositionID p_id : op.effects){
            graph.add_edge(p_id, effect_node);
        }
        graph.add_edge(effect_node, operator_node);
        */
       
    
        for (PropositionID p_id : op.preconditions){
            graph.add_edge(operator_node, p_id);
        }

        for (PropositionID p_id : op.effects){
            graph.add_edge(p_id, operator_node);
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
    return -1;
}

int RelaxedTaskGraph::ff_cost_of_goal() {
    // TODO: add your code for exercise 2 (e) here.
    return -1;
}

}