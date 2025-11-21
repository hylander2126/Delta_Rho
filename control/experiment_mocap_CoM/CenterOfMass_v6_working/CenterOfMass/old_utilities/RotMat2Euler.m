function Eul = RotMat2Euler(R)

if R(1,3) == 1 | R(1,3) == -1
  %special case
  E3 = 0; %set arbitrarily
  dlta = atan2(R(1,2),R(1,3));
  if R(1,3) == -1
    E2 = pi/2;
    E1 = E3 + dlta;
  else
    E2 = -pi/2;
    E1 = -E3 + dlta;
  end
else
  E2 = - asin(R(1,3));
  E1 = atan2(R(2,3)/cos(E2), R(3,3)/cos(E2));
  E3 = atan2(R(1,2)/cos(E2), R(1,1)/cos(E2));
end

Eul = [E1 E2 E3];