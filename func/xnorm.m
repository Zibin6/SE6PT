function V = xnorm(V0)

	V = V0 .* kron(ones(3,1), 1./sqrt(sum(V0.^2)));

end