function T = read_cfd_data(dataFile)
%READ_CFD_DATA  读取 CFD 表，删除 model / alpha / beta 列
%
% T = read_cfd_data()
% T = read_cfd_data('CFD_DATA.txt')
%
% 默认读取本脚本同目录下的 CFD_DATA.txt。
% 清洗后的表写入 CFD_DATA_clean.txt 和 CFD_DATA_clean.mat。

    if nargin < 1 || isempty(dataFile)
        dataFile = fullfile(prop_paths().lib, 'CFD_DATA.txt');
    end

    if ~isfile(dataFile)
        error('找不到数据文件: %s', dataFile);
    end

    info = dir(dataFile);
    if info.bytes == 0
        error('数据文件是空的 (0 字节): %s', dataFile);
    end

    opts = detectImportOptions(dataFile, 'FileType', 'text');
    if any(strcmp(properties(opts), 'VariableNamingRule'))
        opts.VariableNamingRule = 'preserve';
    end
    T = readtable(dataFile, opts);

    % 匹配 model / alpha / beta，含 "ALPHA DEG"、"BETA DEG" 这类带单位的列名
    dropNames = {'model', 'alpha', 'beta'};
    vars = T.Properties.VariableNames;
    dropMask = false(1, numel(vars));
    for k = 1:numel(vars)
        dropMask(k) = is_drop_column(vars{k}, dropNames);
    end

    if ~any(dropMask)
        warning('未找到 model / alpha / beta 列。当前列名:\n  %s', strjoin(vars, ', '));
    else
        fprintf('删除列: %s\n', strjoin(vars(dropMask), ', '));
        T(:, dropMask) = [];
    end

    fprintf('剩余 %d 行 × %d 列:\n', height(T), width(T));
    disp(T.Properties.VariableNames);

    io = prop_paths();
    outTxt = fullfile(io.results, 'CFD_DATA_clean.txt');
    outMat = fullfile(io.results, 'CFD_DATA_clean.mat');
    writetable(T, outTxt, 'Delimiter', '\t');
    save(outMat, 'T');
    fprintf('已写入:\n  %s\n  %s\n', outTxt, outMat);
end

function tf = is_drop_column(name, dropNames)
    name = lower(char(string(name)));
    compact = regexprep(name, '[^a-z0-9]', '');
    words = regexp(name, '[a-z]+', 'match');
    tf = false;
    for i = 1:numel(dropNames)
        d = dropNames{i};
        if strcmp(compact, d) || startsWith(compact, [d 'deg'])
            tf = true;
            return
        end
        if ~isempty(words) && strcmp(words{1}, d)
            tf = true;
            return
        end
    end
end
