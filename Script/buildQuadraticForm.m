%%%%%%%%%%%%%%%%%%%%
% Takes a 1-d expression and delivers the middle tensor of a quadratic
% form
%%%%%%%%%%%%%%%%%%%%

%%
function SymMatrix = buildQuadraticForm(f,dr)

% ----------- Test ------------------
% clc
% TestMat = [2,3*sd,1;3,5*sp,0;1,0,6]
% f = dr'*TestMat*dr;
% -----------------------------------
% f = d_Mr_rdot
nn = length(f);
mm = length(dr);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tensor Order: dr_row dr_column f
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SymMatrix = sym(zeros(mm,mm,nn));

%-------- ignore the for ---------
% k = 1;
%---------------------------------

% f_rew1 = zeros(n);
% for k = 1:n
%     f_rew1(k) = collect(f(k),dr);
% end

%%
% clc
MatC = sym(zeros(nn,sum(1:mm)));
MatT = MatC;
for k = 1:nn
    [tmpC,tmpT] = coeffs(f(k),dr);
    MatC(k,1:length(tmpC)) = tmpC;
    MatT(k,1:length(tmpT)) = tmpT;
end

tic
for k = 1:nn
    for ii = 1:mm
        for jj = 1:ii
            if ~isempty( MatC(k,MatT(k,:) == dr(ii)*dr(jj)) )
                MyFac = 0.5; % Non-Diagonal
                if ii == jj % Diagonal
                    MyFac = 1;
                end
                    SymMatrix(ii,jj,k) = MyFac*MatC(k,MatT(k,:)==dr(ii)*dr(jj));
                    SymMatrix(jj,ii,k) = SymMatrix(ii,jj,k);
            end
        end
    end
end
t_calc = toc;
% disp(['--In Fcn BuildquadraticForm: *** Time for calculating: ',num2str(t_calc) ' ***'])
tic
SymMatrix = simplify(SymMatrix,   ...
                'IgnoreAnalyticConstraints', true,'Steps', 0);
t_calc = toc;
% disp(['--In Fcn BuildquadraticForm: *** Time for simplification: ',num2str(t_calc) ' ***'])


