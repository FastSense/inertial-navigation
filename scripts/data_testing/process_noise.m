% perform noise computations on given data set
% input:
%     RawData  --- vector of data
%     fs       --- sample rate
%     freqBand --- (optional) two element array of frequency band for RND calculation;
%                  defaults to [10,40] for 10Hz - 40Hz band
% output:
%     freq --- frequency Vector [Hz]
%     PSD  --- vector of PSD output [units^2/rtHz]
%     RMS  --- single value for overall RMS noise [units rms]
%     RND  --- single value for overall RND [units/rtHz]
% references:
%     A.Cantwell, Analog Devices, 4/29/2012
function [freq, PSD, RMS, RND] = process_noise(RawData,fs,freqBand)

    if(~exist('freqBand'))
        freqBand(1) = 10;
        freqBand(2) = 40;
    elseif(freqBand(1)>=freqBand(2))
        error('Lower frequency bound must be less than upper bound')
    end
    
    % remove mean from dataset
    RawData = RawData - mean(RawData);
    
    % compute noise
    RMS = std(RawData);

    % perform PSD computation using PWELCH method.  Output is in [units]^2/Hz
    [PSD,freq] = pwelch(RawData,[],[],[],fs,'onesided');

    %calculate average noise on desired interval
    RND = sqrt(mean(PSD(freq>freqBand(1) & freq<freqBand(2))));

    % convert data from [units]^2/Hz to [units]/rtHz.
    PSD = PSD.^.5;
end