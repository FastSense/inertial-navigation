function [freq,PSD] = smooth_psd(freq_IN,PSD_IN,binsize)
    freq = freq_IN(1:binsize:end-1);
    PSD=sqrt(mean(reshape(PSD_IN(1:floor(length(PSD_IN)/binsize)*binsize).^2,binsize,floor(length(PSD_IN)/binsize))));
end