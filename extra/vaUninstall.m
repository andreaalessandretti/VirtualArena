%% vaUninstall undo vaInstall
% and takes the same parameters as input.

function vaUninstall(url,nameFolder)

if nargin == 2
    
    packDir = nameFolder;
    
else
    
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
    
    
    packDir = zipDir;
    rmdir([tempFolder zipDir],'s');
end

if sum(strcmp(nameFolders,packDir))>0 % ... if yes ask for an other
    
    pathToDelete = [pathstr filesep packDir];
    
    libFolderName = 'lib';
    
    retDir      = dir(pathToDelete);
    nameFolders = {retDir(:).name};
    
    if sum(strcmp(nameFolders,libFolderName))==1
        
        folderToRm = [pathToDelete filesep libFolderName];
        rmpath(folderToRm);
        savepath;
        disp(['Folder ''' folderToRm ''' removed from Matlab path.']);
        
    end
    
    rmdir(pathToDelete,'s');
    
else % ... else put the folder in place
  
    error('Package not found.');
    
end

end