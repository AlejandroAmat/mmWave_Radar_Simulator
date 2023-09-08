function SNRf = simulate_radar_signal(nTx, Tx, reflector_cart_v, T_pos)
% Synth Radar signal

% Simualte received radar signal in the receiver antenna array
% Input: "reflector_cart_v" coordinates of point reflectors in the scene 
% Output: "signal_array" FMCW beat signals in the antenna array

    variable_library_radar; % load radar configurations
        
    t_ax = 0:1/Rs:(Ts-1/Rs); t_ax = t_ax.'; % time axis
    t_matrix = repmat(t_ax,1,array_size(1),array_size(2));
    SNRf = ones(1,length(reflector_cart_v));
    
    signal_array = zeros(length(t_ax),array_size(1),array_size(2));

    for kp = 1:length(reflector_cart_v)
        d_TX2reflector = sqrt((T_pos(1)-reflector_cart_v(kp,1)).^2+(T_pos(2)-reflector_cart_v(kp,2)).^2+ (T_pos(3)-reflector_cart_v(kp,3)).^2);
        % distance between the TX antenna and the point reflector
        %d_RX2reflector = reshape(sqrt((array_x_m-reflector_cart_v(kp,1)).^2 + reflector_cart_v(kp,2)^2 + (array_z_m-reflector_cart_v(kp,3)).^2),[1,array_size(1),array_size(2)]);
        % distance between each RX antenna and the point reflector
      
        tau = repmat((d_TX2reflector)/c,[length(t_ax),1,1]); % round trip Time of Flight (ToF)
        path_loss = repmat(Tx_power/d_TX2reflector,[length(t_ax),1,1]);
        pt_signal = (path_loss.*exp(1j * 2 * pi * fc * tau)).* exp(1j * 2*pi * As * t_ax .*tau); % beat signal from a single point reflector
        
        
        
        
        Pw_signal=  mean(abs(pt_signal(:)).^2);
        Pw_signal = 10*log10(Pw_signal);
        Pw_noise = 10*log10(NoisePW);
        SNRf(kp) = Pw_signal - Pw_noise;
                
          

         
    end
    
    
end