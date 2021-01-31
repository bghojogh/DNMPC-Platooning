function projected_vector = project_onto_subspace(weight_matrix, vector)

    if sum(size(weight_matrix)) > 3
        weight_matrix = reshape(weight_matrix, 2, 2);
    end 
    [U,S,V] = svd(weight_matrix);
    projection_matrix = U * diag(diag(S) .^ 0.5);
    projected_vector = projection_matrix' * vector;
   
end
