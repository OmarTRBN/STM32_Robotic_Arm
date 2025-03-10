% MATLAB Script: CSV to C const uint16_t arrays (LUT_PSC and LUT_ARR)
clc;
clear;

% Read CSV file
data = csvread('LUT.csv');

% Check if data has exactly three columns
if size(data, 2) ~= 3
    error('CSV file must have exactly 3 columns.');
end

% Extract columns (skip the first one)
LUT_PSC = data(:, 2);  % Second column
LUT_ARR = data(:, 3);  % Third column

% Open a file to write the C arrays
outputFile = fopen('LUT_arrays.c', 'w');

% Write the includes
fprintf(outputFile, '#include <stdint.h>\n\n');
fprintf(outputFile, '#define LUT_SIZE %d\n\n', size(data, 1));

% Write LUT_PSC array
fprintf(outputFile, 'const uint16_t LUT_PSC[LUT_SIZE] = { ');
fprintf(outputFile, '%d', LUT_PSC(1));
for i = 2:length(LUT_PSC)
    fprintf(outputFile, ', %d', LUT_PSC(i));
end
fprintf(outputFile, ' };\n\n');

% Write LUT_ARR array
fprintf(outputFile, 'const uint16_t LUT_ARR[LUT_SIZE] = { ');
fprintf(outputFile, '%d', LUT_ARR(1));
for i = 2:length(LUT_ARR)
    fprintf(outputFile, ', %d', LUT_ARR(i));
end
fprintf(outputFile, ' };\n\n');

% Close the file
fclose(outputFile);

disp('C arrays successfully written to LUT_arrays.c');
