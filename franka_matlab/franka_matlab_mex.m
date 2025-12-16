function franka_matlab_mex()
    %  Copyright (c) 2025 Franka Robotics GmbH - All Rights Reserved
    %  This file is subject to the terms and conditions defined in the file
    %  'LICENSE' , which is part of this package

    installation_path = franka_installation_path_get();

    franka_matlab_src_path = fullfile(installation_path,'franka_matlab','src');
    franka_robot_server_path = fullfile(installation_path,'franka_robot_server');
    franka_robot_server_build_path = fullfile(franka_robot_server_path,'build');

    destination_path = fullfile(installation_path,'franka_matlab','build');
    
    if ~isfolder(destination_path)
    	mkdir(destination_path);
    end
    
    opts = struct('verbose', true, 'nothrow', false);
    
    if isunix()
        % Generate capnp files first
        franka_local_exec('./generate_capnp.sh', franka_robot_server_path, opts);
        
        mex_string = strjoin({...
            'mex', ...
            ['-I"',franka_robot_server_build_path,'/interface"'], ...
            '-I"/usr/local/include/capnp/include"', ...
            '-L"/usr/local/lib"', ...
            fullfile(franka_matlab_src_path,'franka_robot.cpp'), ...
            fullfile(franka_robot_server_build_path,'interface','rpc.capnp.c++'), ...
            '/usr/local/lib/libcapnp-rpc.a', ...
            '/usr/local/lib/libcapnp.a', ...
            '/usr/local/lib/libkj-async.a', ...
            '/usr/local/lib/libkj.a', ...
            '-lpthread', ... % Added pthread dependency
            'CXXFLAGS="\$CXXFLAGS -std=c++17 -fPIC"', ...
            [' -outdir ', destination_path]
        });

    elseif ispc()

        % Generate capnp files first
        franka_local_exec('generate_capnp.bat', franka_robot_server_path, opts);
        
        capnproto_installation_dir = 'C:\Program Files (x86)\capnproto-c++-win32-1.0.2\capnproto-c++-1.0.2\src';

        mex_string = strjoin({...
            'mex', ...
            ['-I"',fullfile(franka_robot_server_build_path,'interface'),'"'], ...
            ['-I"',capnproto_installation_dir,'"'], ...
            '-I"/usr/local/include/capnp/include"', ...
            '-L"/usr/local/lib"', ...
            '-L"/usr/local/lib"', ... 
            ['-L"',fullfile(capnproto_installation_dir,'capnp','Release'),'"'], ...
            ['-L"',fullfile(capnproto_installation_dir,'kj','Release'),'"'], ...
            '-lcapnp', ...
            '-lcapnp-rpc', ...
            '-lkj', ...
            '-lkj-async', ...
            '-lWs2_32', ...
            'COMPFLAGS="$COMPFLAGS /std:c++17"', ...
            fullfile(franka_matlab_src_path,'franka_robot.cpp'), ...
            fullfile(franka_robot_server_build_path,'interface','rpc.capnp.cpp'), ...
            [' -outdir ', destination_path]
        });

    end

    eval(mex_string);

    if ~isfolder(fullfile(installation_path,'franka_matlab','bin'))
        mkdir(fullfile(installation_path,'franka_matlab','bin'));
    end

    if isfile(fullfile(installation_path,'franka_matlab','bin.zip'))
        unzip(fullfile(installation_path,'franka_matlab','bin.zip'),fullfile(installation_path,'franka_matlab'));
    end
    
    if isunix()
        produce = 'franka_robot.mexa64';
    elseif ispc()
        produce = 'franka_robot.mexw64';
    end
    copyfile(fullfile(installation_path, 'franka_matlab','build',produce),fullfile(installation_path,'franka_matlab','bin')); 
    
    zip(fullfile(installation_path,'franka_matlab','bin.zip'),fullfile(installation_path,'franka_matlab','bin'));

    if isfolder(franka_robot_server_build_path)
        rmdir(franka_robot_server_build_path, 's');
    end
    if isfolder(destination_path)
        rmdir(destination_path, 's');
    end
    if isfolder(fullfile(installation_path,'franka_matlab','bin'))
        rmdir(fullfile(installation_path,'franka_matlab','bin'), 's');
    end

end
