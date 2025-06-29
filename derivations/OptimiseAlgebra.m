function [SymExpOut,SubExpArray] = OptimiseAlgebra(SymExpIn,SubExpName)
% Loop through symbolic expression, identifying repeated expressions and
% bringing them out as shared expression or sub expressions
% do this until no further repeated expressions found
% This can significantly reduce computations
syms SubExpIn SubExpArray;
SubExpArray(1,1) = 'invalid';
index = 0;
f_complete = 0;
while f_complete==0
    index = index + 1;
    
    % Create the symbolic variable name
    varName = [SubExpName,'_',num2str(index),'_'];  % Use underscore instead of parentheses
    
    % Create the symbolic variable dynamically
    eval(['syms ', varName, ' real']);
    
    % Store the variable name for comparison
    SubExpInStore{index} = varName;
    
    % Get the symbolic variable
    SubExpIn = eval(varName);
    
    % Find sub-expressions
    [SymExpOut,SubExpOut]=subexpr(SymExpIn,SubExpIn);
    
    % Check if we found any new sub-expressions
    f_complete = 1;  % Assume we're done
    if ~isequal(SymExpOut, SymExpIn)  % If something changed
        f_complete = 0;  % We're not done yet
        SubExpArray(index,1) = SubExpOut;
        SymExpIn = SymExpOut;
    end
    
    % Safety check to prevent infinite loop
    if index > 100
        f_complete = 1;
        SymExpOut = SymExpIn;
    end
end
end