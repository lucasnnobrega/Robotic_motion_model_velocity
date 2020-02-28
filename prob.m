function resultado = prob(a,b)
    expoente = (-1/2)*(a^2/b^2);
    
    resultado = (1/sqrt(2*pi*b^2))* exp(expoente); 
end