function t = rnfield(s,f1,f2)

% RNFIELD Rename structure field.
%
%   S = RNFIELD(S,OldField,NewField) renames the specified field 
%        in the structure array S.  
%
%   S = RNFIELD(S,OldFields,NewFields) renames more than one field 
%   at a time when Fields is a character array or cell array of strings.
%
%   The changed structure is returned.
%
%   See also RMFIELD, INSFIELD, SETFIELD, GETFIELD, FIELDNAMES

%   C.Begler IFM-GEOMAR Kiel


if nargin < 3
   error('Not enough InputArguments.');
end

if ~isa(s,'struct'), error('S must be a structure array.'); end

if ischar(f1), f1 = cellstr(f1); end
if ischar(f2), f2 = cellstr(f2); end

if ~( iscellstr(f1) & iscellstr(f2) )
   error('FIELDNAMES must be a cell array of strings.');
elseif ~isequal(size(f1),size(f2))
   error('Size of FIELDNAMES must be agree.');
end

f0 = fieldnames(s); % Org
f3 = f0;            % New

% Determine which fieldnames to rename.
n = prod(size(f1));

ok = zeros(1,n);

for ii = 1 : n
    jj = strmatch(f1{ii},f0,'exact');
    ok(ii) = ~isempty(jj);
    if ok(ii)
       f3(jj) = f2(ii);
    end
end

if ~all(ok)
    jj = find(~ok);
    m = sprintf(' "%s" ',f1{jj});
    warning(sprintf('FieldNames doesn''t exist.\n%s',m));
    if ~any(ok)
        t = s;
        return
    end
end

f = double(char(sort(f3)));
f = ( sum( diff(f,1,1) == 0 , 2 ) == size(f,2) );
if any(f)
   jj = find(f);
    f = sort(f3);
    m = sprintf(' "%s" ',f{jj});
   warning(sprintf('Duplicate FieldNames.\n%s',m));
end

% Now rename the fieldnames by copying the ones we want to keep.
t = [];
for ii = prod(size(s)) : -1 : 1,
    for jj = 1 : prod(size(f3)),
         t = setfield(t,{ii},f3{jj},getfield(s,{ii},f0{jj}));
    end
end

if ~isempty(t)
    t = reshape(t,size(s));
end
