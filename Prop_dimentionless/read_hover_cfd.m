function T = read_hover_cfd(dataFile)
%READ_HOVER_CFD  读取举升桨 CFD_Hover.txt
%
% T = read_hover_cfd()
% T = read_hover_cfd('CFD_Hover.txt')
%
% 丢掉 MODEL、TILT PROP RPM（全是 NaN）。
% 攻角 / 侧滑保留：这张表 α 不是全 0，会进 Ja。

    if nargin < 1 || isempty(dataFile)
        dataFile = fullfile(fileparts(mfilename('fullpath')), 'CFD_Hover.txt');
    end
    if ~isfile(dataFile)
        error('找不到数据文件: %s', dataFile);
    end

    opts = detectImportOptions(dataFile, 'FileType', 'text');
    if any(strcmp(properties(opts), 'VariableNamingRule'))
        opts.VariableNamingRule = 'preserve';
    end
    T = readtable(dataFile, opts);

    vars = T.Properties.VariableNames;
    dropMask = false(1, numel(vars));
    for k = 1:numel(vars)
        compact = regexprep(lower(char(string(vars{k}))), '[^a-z0-9]', '');
        dropMask(k) = strcmp(compact, 'model') || startsWith(compact, 'tiltproprpm');
    end
    if any(dropMask)
        fprintf('删除列: %s\n', strjoin(vars(dropMask), ', '));
        T(:, dropMask) = [];
    end

    fprintf('剩余 %d 行 × %d 列:\n', height(T), width(T));
    disp(T.Properties.VariableNames);

    here = fileparts(dataFile);
    if isempty(here)
        here = pwd;
    end
    outTxt = fullfile(here, 'CFD_HOVER_clean.txt');
    outMat = fullfile(here, 'CFD_HOVER_clean.mat');
    writetable(T, outTxt, 'Delimiter', '\t');
    save(outMat, 'T');
    fprintf('已写入:\n  %s\n  %s\n', outTxt, outMat);
end
