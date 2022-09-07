function channel = create_chan(SampleRate,OperatingFrequency)

    channel = phased.FreeSpace(...
        'SampleRate',SampleRate,...
        'TwoWayPropagation',true,...
        'OperatingFrequency',OperatingFrequency);
end