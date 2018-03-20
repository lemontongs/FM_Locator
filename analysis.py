
from matplotlib import pyplot
import pandas
import numpy

print "read CSV"
x = pandas.read_csv('test_107100000.csv')

print "to complex"
sig = [complex(x.values[i,0], x.values[i,1]) for i in range(x.shape[0])]

print "fft"
F = 2**int(numpy.ceil(numpy.log2(len(sig))))
print F

fs = 500e3

sig_fft = 20.0*numpy.log10(numpy.abs(numpy.fft.fftshift(numpy.fft.fft(sig, F))))
f_bins = numpy.fft.fftshift(numpy.fft.fftfreq(F, 1.0/fs))
sig_mask = numpy.bitwise_and(f_bins > -75000, f_bins < 75000)
noise_mask = numpy.bitwise_and(f_bins > -100000, f_bins < 100000)
noise_mask = numpy.bitwise_xor(noise_mask, sig_mask)
sig_mean = numpy.mean(sig_fft[sig_mask])
noise_mean = numpy.mean(sig_fft[noise_mask])

print sig_mean, noise_mean

#print "running average"
#N = 10000
#average = numpy.convolve(sig_fft, numpy.ones((N,))/N, mode='valid')

print "plot"
pyplot.plot(f_bins[::10], sig_fft[::10])
pyplot.plot([-fs/2, fs/2], [sig_mean, sig_mean], 'm')
pyplot.plot([-fs/2, fs/2], [noise_mean, noise_mean], 'r')
pyplot.show()

