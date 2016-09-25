% Author: Ofir Weisse, mail: oweisse (at) umich.edu, www.ofirweisse.com
%
% MIT License
%
% Copyright (c) 2016 oweisse
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
%
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

classdef FFTnalyzer < handle
    properties (Constant)
        BASE_DIR                = 'consecutive_sweeps';
    end

    properties
        mostRecentDir;
        rawIQ;
        bandwidth;
        samplingRate;
        centerFrequency;
        stftVals;
        scannedFrequencies;
        tPower;
        relevantFrequenciesIds;
        effectiveScannedFrequencies;
        powerDensity;
        ffaRightHandFrequenciesIds;
        effectiveFFAFrequencies;
    end

    methods
        function obj = FFTnalyzer()
            obj.LoadRecentDir()

            obj.plottedFFA_frequencies = [];
            obj.plottedFFA_values      = [];
        end

        function [ chunkIdx ] = LoadRawIQFile_byFrequency( obj, frequencyHz )
            [ lastCompleteSweep, ~ ] = PollStatus( obj.mostRecentDir );

            for chunkIdx = 0:lastCompleteSweep
                sweepFilePath = sprintf( '%s/%s/rawIQ%04d.bin', ...
                                         obj.BASE_DIR,          ...
                                         obj.mostRecentDir{:},   ...
                                         chunkIdx );
                fprintf( 'Processing file %s\n', sweepFilePath );

                [ obj.bandwidth,    ...
                  obj.samplingRate, ...
                  obj.centerFrequency ] = obj.ParseBinaryIQFileHeader( sweepFilePath );

                startFrequency = obj.centerFrequency - obj.bandwidth/2;
                endFrequency   = obj.centerFrequency + obj.bandwidth/2;

                if frequencyHz >= startFrequency && frequencyHz <= endFrequency
                    [ obj.rawIQ,        ...
                      obj.bandwidth,    ...
                      obj.samplingRate, ...
                      obj.centerFrequency ] = obj.ParseBinaryIQFile( sweepFilePath );
                    return;
                end
            end

            %no capture with requested frequency
            chunkIdx = Inf;
        end

        function [] = LoadRawIQFile_byIndex( obj, chunkIdx )
            sweepFilePath = sprintf( '%s/%s/rawIQ%04d.bin', ...
                                     obj.BASE_DIR,          ...
                                     obj.mostRecentDir{:},   ...
                                     chunkIdx );
            fprintf( 'Processing file %s\n', sweepFilePath );

            [ obj.rawIQ,        ...
              obj.bandwidth,    ...
              obj.samplingRate, ...
              obj.centerFrequency ] = ParseBinaryIQFile( sweepFilePath );
        end

        function LoadRecentDir( obj )
            consecutiveSweepsDirs   = dir( [obj.BASE_DIR '/*'] );
            sortedDirs              = sort( { consecutiveSweepsDirs.name} );
            obj.mostRecentDir       = sortedDirs( end );
        end

        function [] = ComputeTimeDomainFFT( obj, windowLen, nfftPowerDensity )
            overlapSize         = windowLen / 2;
            hopSize             = windowLen - overlapSize;
            tic;
            [obj.stftVals,                                       ...
             obj.scannedFrequencies,                             ...
             obj.tPower]                = stft( obj.rawIQ,       ...
                                                windowLen,       ...
                                                hopSize,         ...
                                                nfftPowerDensity,...
                                                obj.samplingRate);
            toc;

            obj.CalcRelevantFrequenciesIDs()

            tic;
            obj.powerDensity = 2*10*log10( abs(                           ...
                            obj.stftVals( obj.relevantFrequenciesIds, : ) ...
            ) ); %abs^2 was converted to 2* outside the logarithm
            toc
        end

        function CalcRelevantFrequenciesIDs( obj )
            relevantFrequenciesFraction      = obj.bandwidth / obj.samplingRate;
            excessFrequenciesFraction        = 1 - relevantFrequenciesFraction;
            relevantFrequenciesStartFraction = excessFrequenciesFraction / 2;
            relevantFrequenciesEndFraction   = 1 - relevantFrequenciesStartFraction;
            numberOfFrequencies              = length( obj.scannedFrequencies );
            obj.relevantFrequenciesIds       = ...
                floor( numberOfFrequencies * relevantFrequenciesStartFraction ) : ...
                ceil( numberOfFrequencies  * relevantFrequenciesEndFraction ) ;

            obj.effectiveScannedFrequencies = ...
                obj.scannedFrequencies(obj.relevantFrequenciesIds) + ...
                obj.centerFrequency;
        end

        function [] = PlotSpecrumScan( obj, sweepIdxToPlot )
            plot( obj.effectiveScannedFrequencies,          ...
                  obj.powerDensity( obj.relevantFrequenciesIds, sweepIdxToPlot ) );
            set(gca,'FontSize',40);
            grid on;
        end


        function PlotMaxMinEnergy( obj, offset )
            OPERATE_ON_COLUMNS = 2;

            if nargin < 2
               offset = 0;
            end

            minEnergyVals = min( obj.powerDensity, [], OPERATE_ON_COLUMNS );
            maxEnergyVals = max( obj.powerDensity, [], OPERATE_ON_COLUMNS );

            plot(  obj.effectiveScannedFrequencies, ...
                   [ minEnergyVals, maxEnergyVals ]' + offset );
        end

        function AnalyzeContiniousScan( obj )
            windowLen                   = 2^14;
            nfftPowerDensity            = 2^14;
            minHz                       = Inf;
            maxHz                       = 0;
            obj.LoadRecentDir()

            doneSweeping        = false;
            lastCompleteSweep   = -1;
            currentPartialSweep = 0;
            while ~doneSweeping || currentPartialSweep <= lastCompleteSweep
                while currentPartialSweep > lastCompleteSweep && ~doneSweeping
                    [ lastCompleteSweep, doneSweeping ] = PollStatus( obj.mostRecentDir );
                    pause( 0.1 );
                end

                obj.LoadRawIQFile_byIndex   ( currentPartialSweep );
                obj.ComputeTimeDomainFFT    ( windowLen, nfftPowerDensity );

                plot(  obj.effectiveScannedFrequencies, ...
                       obj.powerDensity );

                minHz = min( currentMinHz, minHz );
                maxHz = max( currentMaxHz, maxHz );
                xlim( [ minHz, maxHz ] );
                hold on;

                refreshdata;
                pause(0.0001);

                currentPartialSweep = currentPartialSweep + 1;
            end

            disp( 'Done!' );
        end



    end

    methods(Static)
         function [ rawIQ,        ...
                    bandwidth,    ...
                    samplingRate, ...
                    centerFrequency ] = ParseBinaryIQFile( filePath )

            fid = fopen( filePath );

            %%
            bandwidth       = fread( fid, 1, 'float' );
            samplingRate    = fread( fid, 1, 'float' );
            totalSamples    = fread( fid, 1, 'float' );
            centerFrequency = fread( fid, 1, 'float' );
            %%
            rawVals         =  fread( fid,[totalSamples], '*float' );
            %%
            fclose( fid );

            %%
            Is = rawVals( 1:2:end );
            Qs = rawVals( 2:2:end );
            rawIQ = Is + 1i * Qs;
         end

         function [   bandwidth,    ...
                      samplingRate, ...
                      centerFrequency ] = ParseBinaryIQFileHeader( filePath )
            fid = fopen( filePath );

            %%
            bandwidth       = fread( fid, 1, 'float' );
            samplingRate    = fread( fid, 1, 'float' );
                              fread( fid, 1, 'float' ); %don't need "total samples" , as we are only interested in the meta data
            centerFrequency = fread( fid, 1, 'float' );

            %%
            fclose( fid );
        end
    end
end
