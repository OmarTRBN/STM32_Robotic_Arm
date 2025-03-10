% Clock frequency
clock_freq = 50e6;

min_freq = 1;
max_freq = 5e3;

frequencies = min_freq:1:max_freq;
arr_values = zeros(size(frequencies));
psc_values = zeros(size(frequencies));

% Iterate over each frequency
for i = 1:length(frequencies)
    % Desired frequency
    freq_des = frequencies(i) * 2;
    
    if freq_des == 0
        arr_values(i) = 0;
        psc_values(i) = 0;
    else
        % Calculate prescaler (PSC) and auto-reload register (ARR)
        for psc = 0:65535
            arr = round(clock_freq / ((psc+1) * freq_des)) - 1;
            if arr >= 0 && arr <= 65535  % Check for valid ARR value
                arr_values(i) = arr;
                psc_values(i) = psc;
                break;
            end
        end
    end
end

fprintf('Frequency (Hz)  |  ARR  |  PSC\n');
fprintf('----------------|-------|-------\n');
for i = 1:length(frequencies)
    fprintf('%13.0f  |  %4d  |  %4d\n', frequencies(i), arr_values(i), psc_values(i));
end

% Create the Look-Up Table (LUT) as a matrix [Frequency, PSC, ARR]
LUT = [frequencies', psc_values', arr_values'];

% Save LUT to a CSV file for easy integration into STM32 code
csvwrite('LUT.csv', LUT);




