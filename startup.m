% When Matlab starts up, it looks for a file called startup.m in the
% directory (folder) in which it is being started.  If such a file is
% found, then it is automatically executed as part of Matlab's start-up
% procedure.  The line at the end of this file tells Matlab to add the
% bullet java library to the dynamic path of java libraries being
% referenced by Matlab.

% WHAT YOU MUST DO:
% If you have a "startup.m" file already, copy this line to the end of it
% and save. If you do NOT have a "startup.m" file in your Matlab directory,
% then copy/move this entire file to that main directory. Please note that
% the file path is listed relative to your Matlab directory. So if you copy
% the mBullet file to /Users/Matt/documents/MATLAB/mBullet, then the file
% path will be as follows:

javaaddpath mBullet/jbullet.jar