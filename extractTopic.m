function [topicData, t_topic] = extractTopic(topics, topicName, instanceID)
%EXTRACTTOPIC Extract a ULog topic's data table and relative time vector.
%
%   [topicData, t_topic] = extractTopic(topics, topicName, instanceID)
%
%   Inputs:
%       topics      - struct/table with fields TopicNames, InstanceID,
%                     TopicMessages (as produced by your ULog import)
%       topicName   - string/char, e.g. 'radio_status', 'airspeed'
%       instanceID  - (optional) integer instance to extract, default 0
%
%   Outputs:
%       topicData   - the table/struct of messages for that topic+instance
%       t_topic     - duration array, seconds from first sample in that topic
%
%   Example:
%       [radioData, t_radio] = extractTopic(topics, 'radio_status');
%       [airspdData, t_airspd] = extractTopic(topics, 'airspeed', 0);

if nargin < 3
    instanceID = 0;
end

idx = strcmp(topics.TopicNames, topicName) & topics.InstanceID == instanceID;

if ~any(idx)
    error('extractTopic:notFound', ...
        'Topic "%s" with InstanceID %d not found in log.', topicName, instanceID);
end

topicData = topics.TopicMessages{idx};
t_topic = seconds(topicData.timestamp - topicData.timestamp(1));
end