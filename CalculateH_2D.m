function val = CalculateH_2D(node_index)
global goal_idx
val = sum(abs(node_index - [goal_idx(1), goal_idx(2)])) + 0.0001 * randn;
end