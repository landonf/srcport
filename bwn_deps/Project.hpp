//  Created by Landon Fuller on 9/6/16.
//  Copyright (c) 2016 Landon Fuller. All rights reserved.

#pragma once

#include <string>
#include <vector>

class Path {
    Path (const std::string &string) : _str(string) {}
    
    const std::string &stringValue () { return _str; }
    
private:
    std::string _str;
};

class Project {
    Project (const std::vector<Path> sourcePaths, const std::vector<Path> targetPaths) : _sourcePaths(sourcePaths), _targetPaths(targetPaths) {
    }

    const std::vector<Path> &targetPaths () { return _targetPaths; }
    const std::vector<Path> &sourcePaths () { return _sourcePaths; }

private:
    std::vector<Path> _sourcePaths;
    std::vector<Path> _targetPaths;
};