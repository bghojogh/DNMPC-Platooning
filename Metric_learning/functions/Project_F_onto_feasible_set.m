function F = Project_F_onto_feasible_set(F, O, car_index, G_set, G_set_cutin)

    sum_G = 0;
    if isempty(O{car_index})
        % F = ProjectOntoPositiveSemideinite(F);
        F = ProjectOntoPositiveSemideinite_epsilon(F, 0.01);
    else
        for index = 1:length(O{car_index})
            recieving_car_index = O{car_index}(index);
            if mod(recieving_car_index, 1) == 0  %--> if sending information to an original car, not a cut-in car
                sum_G = sum_G + reshape(G_set{recieving_car_index}, 2, 2);
            else
                recieving_car_index_cutin = recieving_car_index + 0.5;
                sum_G = sum_G + reshape(G_set_cutin{recieving_car_index_cutin}, 2, 2);
            end
        end
        F = ProjectOntoPositiveSemideinite(F - sum_G) + sum_G;
    end
   
end
