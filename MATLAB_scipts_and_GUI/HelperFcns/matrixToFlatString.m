function flatStr = matrixToFlatString(matrix)
    % Convert a matrix to a flat comma-delimited string without brackets
    % Example: [1,2;3,4] becomes "1,2,3,4"
    
    % Transpose and reshape to ensure row-major order (easier for STM32 parsing)
    flatMatrix = reshape(matrix', 1, []);
    
    % Convert to string with commas
    flatStr = sprintf('%.1f,', flatMatrix);
    
    % Remove trailing comma
    flatStr = flatStr(1:end-1);
end