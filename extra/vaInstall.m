%% vaInstall installs an external package
%
%   vaInstall(url) Installs the package from url, which is the URL of a 
%   zip file containing the package files.
%
%   The content of the archive should be structured as follows:
%
%   ./PackageName       contains all the files of the package
%   ./PackageName/lib   contains the libraries of the package that are
%                       added to matlab path 
%
%   By defoult the folder PackageName is copied in /VirtualArena/extra and 
%   /PackageName/lib is added to the path
%
%
%   vaInstall(url,alternativePackageName) instead of using the name of the 
%   folder ./PackageName, it uses alternativePackageName, i.e., the files
%   are copied in /VirtualArena/extra/alternativePackageName. 
%   This is needed in case the name is already used by another package.
%
%   See also vaUninstall
%
function vaInstall(url,nameFolder)

[pathstr,name,ext] = fileparts( which('vaInstall.m') );

tempFolder  = [ pathstr filesep 'temp' filesep] ;

try
    ncmFiles    = unzip(url, tempFolder);
catch err
    error('Error finding/unpacking the desired package.');
end

% Extract package main folder zipDir
aFileName   = ncmFiles{1};
subPath     = aFileName(length(tempFolder)+1:end);
dirsPackage = strsplit(subPath,filesep);
zipDir      = dirsPackage{1};

% Check if the name is already in use ...
retDir      = dir(pathstr);
nameFolders = {retDir(:).name};

if nargin == 2
    packDir = nameFolder;
else
    packDir = zipDir;
end

if sum(strcmp(nameFolders,packDir))>0 % ... if yes ask for an other
    error('The name of the directory already exists, use vaInstall(url,nameFolder) to choose a different nameFolder.');
else % ... else put the folder in place
    
    fromDir = [tempFolder  zipDir];
    toDir   = [pathstr filesep packDir];
    if not(strcmp(fromDir,toDir))
        movefile(fromDir,toDir);
    end
    
end

%% Add to path the lib folder
libFolderName = 'lib';

retDir      = dir(toDir);
nameFolders = {retDir(:).name};

if sum(strcmp(nameFolders,libFolderName))==1
    folderToAdd = [toDir filesep libFolderName];
    addpath(folderToAdd);
    savepath;
    disp(['Folder ''' folderToAdd ''' added to path.']);

else
    error('The package does not contain any lib folder.')
end

end