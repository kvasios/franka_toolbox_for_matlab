function franka_mltbx_package(options)
    %FRANKA_TOOLBOX_DIST_MAKE Create distribution package for Franka Toolbox
    %
    %   franka_mltbx_package() - Full build with git clean (for local dev)
    %   franka_mltbx_package('Mode', 'ci') - CI mode: skip git clean (for GitHub Actions)
    %   franka_mltbx_package('Mode', 'ci', 'OutputName', 'franka') - Specify output name
    %
    %   Options:
    %     Mode       - 'local' (default) or 'ci'
    %     OutputName - Base name for output file (default: 'franka')
    %                  Output will be: dist/<OutputName>.mltbx
    %
    %   Copyright (c) 2024 Franka Robotics GmbH - All Rights Reserved
    %   This file is subject to the terms and conditions defined in the file
    %   'LICENSE' , which is part of this package
    
    arguments
        options.Mode {mustBeMember(options.Mode, {'local', 'ci'})} = 'local'
        options.OutputName {mustBeTextScalar} = 'franka'
    end
    
    ci_mode = strcmp(options.Mode, 'ci');
    output_name = options.OutputName;

    % Determine project root from this script's location
    % (this file lives under scripts/packaging/)
    script_dir = fileparts(mfilename('fullpath'));
    project_root = fullfile(script_dir, '..', '..');
    if ~isfile(fullfile(project_root, 'franka_toolbox.prj'))
        error('franka_mltbx_package:InvalidProjectRoot', ...
            'Could not locate franka_toolbox.prj at project root: %s', project_root);
    end
    
    %% Clean-up env
    % In CI mode, preserve existing .mltbx files (for multi-package builds)
    dist_dir = fullfile(project_root, 'dist');
    dist_content_dirname = 'franka_toolbox_dist_content';
    if ci_mode && exist(dist_dir, 'dir')
        fprintf('CI mode: preserving existing .mltbx files in dist/\n');
        % Only remove the distribution content subfolder, keep .mltbx files
        rm_dir(fullfile(dist_dir, dist_content_dirname));
    else
        rm_dir(dist_dir);
    end
    
    % Check if we're in a git repository before running git clean
    % Skip in CI mode to preserve downloaded artifacts
    if ~ci_mode && exist(fullfile(project_root, '.git'), 'dir')
        fprintf('Running git clean (local mode)...\n');
        system(['cd ',project_root,' && git clean -ffxd']);
    else
        if ci_mode
            fprintf('CI mode: skipping git clean to preserve artifacts\n');
        else
            fprintf('Not in a git repository, skipping git clean\n');
        end
    end

    %% Copy the Project
    target_dir = fullfile(dist_dir, dist_content_dirname);
    
    % Create dist directory if it doesn't exist
    if ~exist(dist_dir, 'dir')
        mkdir(dist_dir);
    end
    
    % Copy project files to distribution directory, excluding dist folder
    copy_project_files(project_root, target_dir);

    %% Remove build and other artifacts
    rm_dir(fullfile(target_dir,'build'));
    rm_dir(fullfile(target_dir,'cmake'));
    rm_dir(fullfile(target_dir,'libfranka'));
    rm_dir(fullfile(target_dir,'libfranka_arm'));
    rm_dir(fullfile(target_dir,'docker'));
    rm_dir(fullfile(target_dir,'.github'));
    safe_delete(fullfile(target_dir,'.gitignore'));
    safe_delete(fullfile(target_dir,'CHANGELOG.md'));
    safe_delete(fullfile(target_dir,'README.md'));
    safe_delete(fullfile(target_dir,'LICENCE'));
    safe_delete(fullfile(target_dir,'LICENSE'));
    safe_delete(fullfile(target_dir,'scripts','packaging','franka_mltbx_package.m'));
    
    remove_all_files_of_type_recursively('.asv',target_dir,{''});

    %% Make the Franka Toolbox for MATLAB
    addpath(genpath(dist_dir));

    fprintf('Packaging toolbox as %s.mltbx...\n', output_name);
    matlab.addons.toolbox.packageToolbox(fullfile(target_dir,'franka_toolbox.prj'),fullfile(project_root,'dist',output_name))
    fprintf('Toolbox packaged successfully: dist/%s.mltbx\n', output_name);

    rmpath(genpath(dist_dir));

end

%% Utilities
function copy_project_files(source_dir, target_dir)
    % Create target directory if it doesn't exist
    if ~exist(target_dir, 'dir')
        mkdir(target_dir);
    end
    
    % Get all items in the source directory
    items = dir(source_dir);
    
    % Remove current and parent directory entries
    items = items(~ismember({items.name}, {'.', '..'}));
    
    % Copy each item, excluding 'dist' folder
    for i = 1:length(items)
        item = items(i);
        if strcmp(item.name, 'dist')
            continue; % Skip the dist directory
        end
        
        source_path = fullfile(source_dir, item.name);
        target_path = fullfile(target_dir, item.name);
        
        % Copy file or directory (copyfile handles both)
        copyfile(source_path, target_path);
    end
end

function rm_dir(dir)
if exist(dir,'dir')
    rmdir(dir,'s');
end
end

function safe_delete(filepath)
if exist(filepath,'file')
    delete(filepath);
end
end

function files = find_all_files_of_type_in_directory_recursively(file_type,directory)
     files = dir(fullfile(directory,'**',['*',file_type]));
end

function remove_all_files_of_type_recursively(file_type,directory,white_list)
    src_files = find_all_files_of_type_in_directory_recursively(file_type,directory);
    src_files(ismember({src_files.name}, white_list)) = [];
    arrayfun(@(c) delete(fullfile(c.folder,c.name)), src_files, 'UniformOutput',false);
end