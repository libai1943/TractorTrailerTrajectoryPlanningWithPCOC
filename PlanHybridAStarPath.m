function [x, y, theta] = PlanHybridAStarPath()
global hybrid_astar_
global vehicle_boundary_value_configs_
global vehicle_physics_

grid_space_ = cell(hybrid_astar_.num_nodes_x, hybrid_astar_.num_nodes_y, hybrid_astar_.num_nodes_theta, hybrid_astar_.num_nodes_theta, hybrid_astar_.num_nodes_theta, hybrid_astar_.num_nodes_theta);
global nominal_goal_config_
nominal_goal_config_ = SpecifyGoalConfig();
init_node = zeros(1,19);
init_node(1,1) = vehicle_boundary_value_configs_.x1_init;
init_node(1,2) = vehicle_boundary_value_configs_.y1_init;
init_node(1,3) = vehicle_boundary_value_configs_.theta1_init;
init_node(1,4) = vehicle_boundary_value_configs_.theta2_init;
init_node(1,5) = vehicle_boundary_value_configs_.theta3_init;
init_node(1,6) = vehicle_boundary_value_configs_.theta4_init;
init_node(1,8) = 0;
init_node(1,9) = CalculateH(init_node(1,1:6));
init_node(1,7) = init_node(1,8) + init_node(1,9);
init_node(1,10) = 0;
init_node(1,11) = 0;
init_node(1,12) = 1;
init_node(1,13) = 0;
init_node(1,14:19) = ones(1,6) .* -999;
openlist_ = init_node;
index = Convert6DimConfigToIndex(init_node(1,1:6));
grid_space_{index(1), index(2), index(3), index(4), index(5), index(6)} = init_node;
expansion_pattern = [1, -vehicle_physics_.phy_max; 1, 0; 1, vehicle_physics_.phy_max; -1, -vehicle_physics_.phy_max; -1, 0; -1, vehicle_physics_.phy_max];
iter = 0;
complete_flag = 0;
complete_via_rs_flag = 0;
min_fitness = Inf;
while ((~isempty(openlist_)) && (iter <= hybrid_astar_.max_iter) && (~complete_flag))
    iter = iter + 1;
    openlist_local_index = find(openlist_(:,7) == min(openlist_(:,7))); openlist_local_index = openlist_local_index(end);
    node_cur = openlist_(openlist_local_index, :);
    node_cur_index = Convert6DimConfigToIndex(node_cur(1,1:6));
    % When the RS path generator is activated ...
    if (mod(iter, hybrid_astar_.num_iters_for_rs) == 1)
        [x_rs, y_rs, theta_rs1, theta_rs2, theta_rs3, theta_rs4] = GenerateRsPathForSearch(node_cur);
        if (Is6DNodeValid(x_rs, y_rs, theta_rs1, theta_rs2, theta_rs3, theta_rs4))
            complete_flag = 1;
            complete_via_rs_flag = 1;
            best_ever_index = node_cur_index;
            break;
        end
    end
    % Remove node_cur from the openlist_ and close it
    openlist_(openlist_local_index, :) = [];
    grid_space_{node_cur_index(1), node_cur_index(2), node_cur_index(3), node_cur_index(4), node_cur_index(5), node_cur_index(6)}(12) = 0;
    grid_space_{node_cur_index(1), node_cur_index(2), node_cur_index(3), node_cur_index(4), node_cur_index(5), node_cur_index(6)}(13) = 1;
    
    for ii = 1 : 6
        from_cur_to_child_v = expansion_pattern(ii,1);
        from_cur_to_child_phy = expansion_pattern(ii,2);
        [x_child, y_child, theta1_child, theta2_child, theta3_child, theta4_child] = SimulateForward(node_cur, from_cur_to_child_v, from_cur_to_child_phy);
        node_child = zeros(1,19);
        node_child(1) = x_child(end);
        node_child(2) = y_child(end);
        node_child(3) = theta1_child(end);
        node_child(4) = theta2_child(end);
        node_child(5) = theta3_child(end);
        node_child(6) = theta4_child(end);
        node_child(10) = from_cur_to_child_v;
        node_child(11) = from_cur_to_child_phy;
        node_child(14:19) = node_cur_index;
        node_child_index = Convert6DimConfigToIndex(node_child(1,1:6));
        
        if ((~isempty(grid_space_{node_child_index(1), node_child_index(2), node_child_index(3), node_child_index(4), node_child_index(5), node_child_index(6)}))...
                &&(grid_space_{node_child_index(1), node_child_index(2), node_child_index(3), node_child_index(4), node_child_index(5), node_child_index(6)}(13) == 1))
            continue;
        end
        if ((isempty(grid_space_{node_child_index(1), node_child_index(2), node_child_index(3), node_child_index(4), node_child_index(5), node_child_index(6)}))||((~isempty(grid_space_{node_child_index(1), node_child_index(2), node_child_index(3), node_child_index(4), node_child_index(5), node_child_index(6)}))...
                &&(grid_space_{node_child_index(1), node_child_index(2), node_child_index(3), node_child_index(4), node_child_index(5), node_child_index(6)}(12) == 0)))
            % If the child node has never been explored
            if (Is6DNodeValid(x_child, y_child, theta1_child, theta2_child, theta3_child, theta4_child))
                if (Is6DNodeCloseToGoal(node_child))
                    complete_flag = 1;
                    best_ever_index = node_child_index;
                    grid_space_{node_child_index(1), node_child_index(2), node_child_index(3), node_child_index(4), node_child_index(5), node_child_index(6)} = node_child;
                    break;
                end
                node_child_fitness = EvaluateFitness(node_child);
                if (node_child_fitness < min_fitness)
                    min_fitness = node_child_fitness;
                    best_ever_index = node_child_index;
                    grid_space_{node_child_index(1), node_child_index(2), node_child_index(3), node_child_index(4), node_child_index(5), node_child_index(6)} = node_child;
                end
                node_child(8) = node_cur(8) + 1 + hybrid_astar_.penalty_multiplier_for_drastic_direction_change * (abs(from_cur_to_child_v - node_cur(10)) + abs(from_cur_to_child_phy - node_cur(11)));
                node_child(9) = CalculateH(node_child(1,1:6));
                node_child(7) = node_child(8) + node_child(9);
                node_child(12) = 1;
                node_child(13) = 0;
                openlist_ = [openlist_; node_child];
            else
                node_child(12) = 0;
                node_child(13) = 1;
            end
            grid_space_{node_child_index(1), node_child_index(2), node_child_index(3), node_child_index(4), node_child_index(5), node_child_index(6)} = node_child;
        else
            % If the child node has already been in the openlist
            node_child_g_attempt = node_cur(8) + 1 + hybrid_astar_.penalty_multiplier_for_drastic_direction_change * (abs(from_cur_to_child_v - node_cur(10)) + abs(from_cur_to_child_phy - node_cur(11)));
            if (grid_space_{node_child_index(1), node_child_index(2), node_child_index(3), node_child_index(4), node_child_index(5), node_child_index(6)}(8) > node_child_g_attempt)
                node_child(8) = node_child_g_attempt;
                node_child(9) = CalculateH(node_child(1,1:6));
                node_child(7) = node_child(8) + node_child(9);
                node_child(12) = 1;
                node_child(13) = 0;
                node_child(14:19) = node_cur_index;
                grid_space_{node_child_index(1), node_child_index(2), node_child_index(3), node_child_index(4), node_child_index(5), node_child_index(6)} = node_child;
                local_child_index = FindOpenlistIndex(openlist_, node_child_index);
                openlist_(local_child_index, :) = [];
                openlist_ = [openlist_; node_child];
            end
        end
    end
end

current_node = grid_space_{best_ever_index(1), best_ever_index(2), best_ever_index(3), best_ever_index(4), best_ever_index(5), best_ever_index(6)};
cur_best_parent_index = current_node(1,14:19);
x = current_node(1,1);
y = current_node(1,2);
theta1 = current_node(1,3);
theta2 = current_node(1,4);
theta3 = current_node(1,5);
theta4 = current_node(1,6);

while (cur_best_parent_index(1) ~= -999)
    current_parrent_node = grid_space_{cur_best_parent_index(1), cur_best_parent_index(2), cur_best_parent_index(3), cur_best_parent_index(4), cur_best_parent_index(5), cur_best_parent_index(6)};
    cur_best_parent_index = current_parrent_node(14:19);
    x = [current_parrent_node(1,1), x];
    y = [current_parrent_node(1,2), y];
    theta1 = [current_parrent_node(1,3), theta1];
    theta2 = [current_parrent_node(1,4), theta2];
    theta3 = [current_parrent_node(1,5), theta3];
    theta4 = [current_parrent_node(1,6), theta4];
end
if (complete_via_rs_flag)
    x = [x, x_rs];
    y = [y, y_rs];
    theta1 = [theta1, theta_rs1];
    theta2 = [theta2, theta_rs2];
    theta3 = [theta3, theta_rs3];
    theta4 = [theta4, theta_rs4];
end
[x2, y2, x3, y3, x4, y4] = ExpandFrom6DimToXY(x, y, theta1, theta2, theta3, theta4);
x = [x; x2; x3; x4];
y = [y; y2; y3; y4];
theta = [theta1; theta2; theta3; theta4];