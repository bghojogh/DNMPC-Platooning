function F_cutin = Project_F_onto_feasible_set_cutin(F_cutin, O_cutin, car_index_cutin, G_set, G_set_cutin)

    sum_G = 0;
    if isempty(O_cutin{car_index_cutin})
        % F = ProjectOntoPositiveSemideinite(F);
        F_cutin = ProjectOntoPositiveSemideinite_epsilon(F_cutin, 0.01);
    else
        for index = 1:length(O_cutin{car_index_cutin})
            recieving_car_index = O_cutin{car_index_cutin}(index);
            if mod(recieving_car_index, 1) == 0  %--> if sending information to an original car, not a cut-in car
                sum_G = sum_G + G_set{recieving_car_index};
            else
                recieving_car_index_cutin = recieving_car_index + 0.5;
                sum_G = sum_G + G_set_cutin{recieving_car_index_cutin};
            end
        end
        F_cutin = ProjectOntoPositiveSemideinite(F_cutin - sum_G) + sum_G;
    end
   
end
