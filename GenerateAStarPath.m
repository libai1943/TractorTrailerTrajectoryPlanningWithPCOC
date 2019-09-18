% ==============================================================================
% MATLAB Source Codes for "Tractor-Trailer Vehicle Trajectory Planning in
% Narrow Environments with a Progressively Constrained Optimal Control Approach". 
% ==============================================================================
function distance = GenerateAStarPath(begin_config, end_config)
global hybrid_astar_
grid_space_2D_ = cell(hybrid_astar_.num_nodes_x, hybrid_astar_.num_nodes_y);
global goal_idx
goal_idx = Convert2DConfigToIndex(end_config);
init_idx = Convert2DConfigToIndex(begin_config);
% Create the node w.r.t. the initial config
g = 0;
h = CalculateH_2D(init_idx);
f = g + h;

global costmap_
% 1x 2y 3f 4g 5h 6is_in_open 7is_in_closed 8-9 parent's index
openlist_ = [init_idx, f, g, h, 1, 0, -999, -999];
grid_space_2D_{init_idx(1), init_idx(2)} = openlist_;
expansion_pattern = [-1 1; -1 0; -1 -1; 0 1; 0 -1; 1 1; 1 0; 1 -1];
expansion_length = [1.414; 1; 1.414; 1; 1; 1.414; 1; 1.414];
iter = 0;
complete_flag = 0;
while ((~isempty(openlist_)) && (iter <= 500) && (~complete_flag))
    iter = iter + 1;
    % Locate the node with smallest f value in the openlist
    current_local_index = find(openlist_(:,3) == min(openlist_(:,3))); current_local_index = current_local_index(end);
    % Name it as cur_node and prepare for extension
    node_current = openlist_(current_local_index, :);
    index_current = node_current(1:2);
    g_current = node_current(4);
    openlist_(current_local_index, :) = [];
    grid_space_2D_{index_current(1), index_current(2)}(6) = 0;
    grid_space_2D_{index_current(1), index_current(2)}(7) = 1;
    
    for ii = 1 : length(expansion_pattern)
        index_child = index_current + expansion_pattern(ii,:);
        if ((index_child(1) < 1)||(index_child(1) > hybrid_astar_.num_nodes_x)||(index_child(2) < 1)||(index_child(2) > hybrid_astar_.num_nodes_y))
            continue;
        end
        
        if ((~isempty(grid_space_2D_{index_child(1), index_child(2)}))&&(grid_space_2D_{index_child(1), index_child(2)}(7) == 1))
            continue;
        end
        if ((isempty(grid_space_2D_{index_child(1), index_child(2)}))||((~isempty(grid_space_2D_{index_child(1), index_child(2)}))&&(grid_space_2D_{index_child(1), index_child(2)}(6) == 0)))
            node_child = zeros(1,9); node_child(1:2) = index_child; node_child(8:9) = index_current;
            % If the child node has never been explored
            if (costmap_(index_child(1), index_child(2)) == 0)
                if (index_child == goal_idx)
                    complete_flag = 1;
                    best_ever_index = index_child;
                    grid_space_2D_{index_child(1), index_child(2)} = node_child;
                    break;
                end
                node_child(4) = g_current + expansion_length(ii);
                node_child(5) = CalculateH_2D(index_child);
                node_child(3) = node_child(4) + node_child(5);
                node_child(6) = 1;
                node_child(7) = 0;
                openlist_ = [openlist_; node_child];
            else
                node_child(6) = 0;
                node_child(7) = 1;
            end
            grid_space_2D_{index_child(1), index_child(2)} = node_child;
        else
            % If the child node has been in the openlist
            node_child_g_attempt = g_current + expansion_length(ii);
            if (grid_space_2D_{index_child(1), index_child(2)}(4) > node_child_g_attempt)
                node_child = grid_space_2D_{index_child(1), index_child(2)};
                node_child(4) = node_child_g_attempt;
                node_child(5) = CalculateH_2D(index_child);
                node_child(3) = node_child(4) + node_child(5);
                node_child(8:9) = index_current;
                grid_space_2D_{index_child(1), index_child(2)} = node_child;
                local_child_index = find(openlist_(:,3) == node_child(3));
                openlist_(local_child_index, :) = [];
                openlist_ = [openlist_; node_child];
            end
        end
    end
end
if (complete_flag)
    counter = 1;
    cur_best_parent_index = grid_space_2D_{best_ever_index(1), best_ever_index(2)}(8:9);
    while (cur_best_parent_index(1) ~= -999)
        counter = counter + 1;
        cur_node = grid_space_2D_{cur_best_parent_index(1), cur_best_parent_index(2)};
        cur_best_parent_index = cur_node(8:9);
    end
    distance = counter * hybrid_astar_.resolution_x * 1.414;
else
    distance = 0;
end