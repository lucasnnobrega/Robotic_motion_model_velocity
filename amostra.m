function r = amostra(b)
r = 0;

 %  You can generate N random numbers in the interval (a,b) with the formula r = a + (b-a).*rand(N,1).
     for i = 1 : 12
        r = r + (-b + (b-(-b)) .* rand(1,1));
     end
end