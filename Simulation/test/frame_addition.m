


 birinci_frame = rotate(x_axis,-pi/2)*rotate(z_axis,135)*trans(1,1,1);
 ikinci_frame = rotate(x_axis,pi/4)*trans(0,-1,1);
 
 birinci_frame_tersi = inv(birinci_frame);
 ikinci_frame_tersi = inv(ikinci_frame);
 
birinci_frame_orijini = hom2vect( birinci_frame_tersi);
birinci_frame_x_birim_vektoru = hom2vect(inv(trans(1,0,0)*birinci_frame ));
birinci_frame_y_birim_vektoru = hom2vect(inv(trans(0,1,0)*birinci_frame ));
birinci_frame_z_birim_vektoru = hom2vect(inv(trans(0,0,1)*birinci_frame ));




ikinci_frame_orijini = hom2vect( ikinci_frame_tersi);
ikinci_frame_x_birim_vektoru = hom2vect(inv(trans(1,0,0)*ikinci_frame ));
ikinci_frame_y_birim_vektoru = hom2vect(inv(trans(0,1,0)*ikinci_frame ));
ikinci_frame_z_birim_vektoru = hom2vect(inv(trans(0,0,1)*ikinci_frame ));


plot3(...
    [birinci_frame_orijini(1)       birinci_frame_x_birim_vektoru(1)],...
    [birinci_frame_orijini(2)       birinci_frame_x_birim_vektoru(2)],...    
    [birinci_frame_orijini(3)       birinci_frame_x_birim_vektoru(3)],'b',...    
    [birinci_frame_orijini(1)       birinci_frame_y_birim_vektoru(1)],...
    [birinci_frame_orijini(2)       birinci_frame_y_birim_vektoru(2)],...    
    [birinci_frame_orijini(3)       birinci_frame_y_birim_vektoru(3)],'r',...    
    [birinci_frame_orijini(1)       birinci_frame_z_birim_vektoru(1)],...
    [birinci_frame_orijini(2)       birinci_frame_z_birim_vektoru(2)],...    
    [birinci_frame_orijini(3)       birinci_frame_z_birim_vektoru(3)],'g')
grid on
    
    