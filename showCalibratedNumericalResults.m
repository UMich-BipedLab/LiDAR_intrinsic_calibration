%{
* Copyright (C) 2013-2025, The Regents of The University of Michigan.
* All rights reserved.
* This software was developed in the Biped Lab (https://www.biped.solutions/) 
* under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
* be available under alternative licensing terms; contact the address above.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
* either expressed or implied, of the Regents of The University of Michigan.
* 
* AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
* WEBSITE: https://www.brucerobot.com/
%}


function results = showCalibratedNumericalResults(calibrated_p2p_distance, original_p2p_distance)
    disp("Showing numerical results...")
    disp("Showing current estimate")
    results = struct('ring', {calibrated_p2p_distance(end).ring(:).ring}, ...
                     'num_points', {calibrated_p2p_distance(end).ring(:).num_points}, ...
                     'mean_original', {original_p2p_distance.ring(:).mean}, ...
                     'mean_calibrated', {calibrated_p2p_distance(end).ring(:).mean}, ...
                     'mean_percentage', num2cell((abs([original_p2p_distance.ring(:).mean]) - abs([calibrated_p2p_distance(end).ring(:).mean])) ./ abs([original_p2p_distance.ring(:).mean])), ...
                     'mean_diff', num2cell([original_p2p_distance.ring(:).mean] - [calibrated_p2p_distance(end).ring(:).mean]), ...
                     'mean_diff_in_mm', num2cell(([original_p2p_distance.ring(:).mean] - [calibrated_p2p_distance(end).ring(:).mean]) * 1e3), ...
                     'std_original', {original_p2p_distance.ring(:).std}, ...
                     'std_calibrated', {calibrated_p2p_distance(end).ring(:).std}, ...
                     'std_diff', num2cell([original_p2p_distance.ring(:).std] - [calibrated_p2p_distance(end).ring(:).std]), ...
                     'std_diff_in_mm', num2cell(([original_p2p_distance.ring(:).std] - [calibrated_p2p_distance(end).ring(:).std])* 1e3));
    struct2table(calibrated_p2p_distance(end).ring(:))
    disp("Showing comparison")
    struct2table(results)
end
