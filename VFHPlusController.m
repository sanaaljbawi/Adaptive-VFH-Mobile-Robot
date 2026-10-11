classdef VFHPlusController < handle
    properties
        num_sectors = 180;
        sector_angle;
        robot_radius = 0.2;
        safety_dist = 0.1;
        min_turn_radius = 0.2;
        threshold_low = 3.0;
        threshold_high = 10.0;
        w_target = 5.0;
        w_current = 2.0;
        w_prev = 2.0;
        prev_selected_angle = 0.0;
    end
    
    methods
        function obj = VFHPlusController()
            obj.sector_angle = 360.0 / obj.num_sectors;
        end
        
        function best_angle = compute_steering(obj, ranges, angles, target_angle, current_angle)
            H_p = zeros(1, obj.num_sectors);
            
            for i = 1:length(ranges)
                r = ranges(i);
                phi = angles(i);
                
                if r <= 0.0 || r > 2.0
                    continue;
                end
                
                m = (1.0 / (r ^ 2)) * (2.0 - r);
                gamma = rad2deg(asin((obj.robot_radius + obj.safety_dist) / r));
                
                for k = 1:obj.num_sectors
                    sector_center = (k-1) * obj.sector_angle + (obj.sector_angle / 2.0);
                    if sector_center > 180
                        sector_center = sector_center - 360;
                    end
                    
                    ang_diff = abs(mod((sector_center - phi + 180), 360) - 180);
                    if ang_diff <= gamma
                        H_p(k) = H_p(k) + m;
                    end
                end
            end
            
            H_b = zeros(1, obj.num_sectors);
            for k = 1:obj.num_sectors
                if H_p(k) > obj.threshold_high
                    H_b(k) = 1;
                elseif H_p(k) < obj.threshold_low
                    H_b(k) = 0;
                else
                    H_b(k) = 1;
                end
            end
            
            H_m = H_b;
            R_min = obj.min_turn_radius + obj.robot_radius;
            
            if target_angle > current_angle
                phi_c = current_angle + 90.0;
            else
                phi_c = current_angle - 90.0;
            end
            phi_c = mod((phi_c + 180), 360) - 180;
            limit_angle = rad2deg(acos(1.0 - (obj.robot_radius^2) / (2.0 * R_min^2)));
            
            for k = 1:obj.num_sectors
                sector_center = (k-1) * obj.sector_angle + (obj.sector_angle / 2.0);
                if sector_center > 180
                    sector_center = sector_center - 360;
                end
                
                if abs(mod((sector_center - phi_c + 180), 360) - 180) < limit_angle
                    H_m(k) = 1;
                end
            end
            
            valleys = [];
            in_valley = false;
            start_idx = 1;
            
            for k = 1:obj.num_sectors
                if H_m(k) == 0 && ~in_valley
                    start_idx = k;
                    in_valley = true;
                elseif H_m(k) == 1 && in_valley
                    valleys = [valleys; start_idx, k - 1];
                    in_valley = false;
                end
            end
            if in_valley
                valleys = [valleys; start_idx, obj.num_sectors];
            end
            
            if isempty(valleys)
                best_angle = current_angle;
                return;
            end
            
            front_sectors_count = 0;
            front_density_sum = 0;
            for k = 1:obj.num_sectors
                sector_center = (k-1) * obj.sector_angle + (obj.sector_angle / 2.0);
                if sector_center > 180
                    sector_center = sector_center - 360;
                end
                if abs(mod((sector_center - target_angle + 180), 360) - 180) <= 45.0
                    front_density_sum = front_density_sum + H_p(k);
                    front_sectors_count = front_sectors_count + 1;
                end
            end
            
            beta = 0;
            if front_sectors_count > 0
                beta = front_density_sum / front_sectors_count;
            end
            
          w_target_adaptive = obj.w_target * exp(-0.15 * beta);
            w_current_adaptive = obj.w_current + (2.0 * beta);

            best_angle = current_angle;
            min_cost = Inf;
            
            for i = 1:size(valleys, 1)
                v_start = valleys(i, 1);
                v_end = valleys(i, 2);
                v_width = v_end - v_start + 1;
                
                if v_width >= 5
                    c_1 = (v_start-1) * obj.sector_angle + (obj.sector_angle / 2.0);
                    c_2 = (v_end-1) * obj.sector_angle + (obj.sector_angle / 2.0);
                    candidates = [c_1, c_2, (c_1 + c_2) / 2.0];
                else
                    candidates = [(((v_start + v_end) / 2.0) - 1) * obj.sector_angle + (obj.sector_angle / 2.0)];
                end
                
                for j = 1:length(candidates)
                    cand = candidates(j);
                    if cand > 180
                        cand = cand - 360;
                    end
                    
                    cost = w_target_adaptive * abs(mod((cand - target_angle + 180), 360) - 180) + ...
                           w_current_adaptive * abs(mod((cand - current_angle + 180), 360) - 180) + ...
                           obj.w_prev * abs(mod((cand - obj.prev_selected_angle + 180), 360) - 180);
                    
                    if cost < min_cost
                        min_cost = cost;
                        best_angle = cand;
                    end
                end
            end
            
            obj.prev_selected_angle = best_angle;
        end
    end
end