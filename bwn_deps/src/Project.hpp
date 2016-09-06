//  Created by Landon Fuller on 9/6/16.
//  Copyright (c) 2016 Landon Fuller. All rights reserved.

#pragma once

#include <string>
#include <vector>

#include <ftl/functional.h>
#include <ftl/vector.h>

class Path {
public:
    Path (const std::string &string) : _str(string) {}
    
    const std::string &stringValue () { return _str; }
    
private:
    std::string _str;
};

class Project {
public:
    Project (const std::vector<std::string> &sourcePaths, const std::vector<std::string> &hostPaths) {
        using ftl::operator%;
        
        const auto &toPath = [](const std::string &str) {
            return Path(str);
        };

        _sourcePaths = toPath % sourcePaths;
        _hostPaths = toPath % hostPaths;
    }
    
    Project (const std::vector<Path> &sourcePaths, const std::vector<Path> &hostPaths) : _sourcePaths(sourcePaths), _hostPaths(hostPaths) {
    }

    const std::vector<Path> &hostPaths () { return _hostPaths; }
    const std::vector<Path> &sourcePaths () { return _sourcePaths; }

private:
    std::vector<Path> _sourcePaths;
    std::vector<Path> _hostPaths;
};