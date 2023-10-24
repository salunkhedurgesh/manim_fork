#!/usr/bin/env python
# coding: utf-8

# In[1]:


from sympy import *
import numpy as np


# In[2]:


var('c1, c2, c3, s1, s2, s3, t1, t2, t3, v1, v2, v3')
var('sa1, sa2, sa3, ca1, ca2, ca3, d1, d2, d3, a1, a2, a3, alpha1 alpha2, alpha3')
var('rho, z')


# In[3]:


c = [c1,c2,c3]
s = [s1,s2,s3]
t = [t1,t2,t3]
v = [v1,v2,v3]
cos_list = [cos(ii) for ii in t]
sin_list = [sin(ii) for ii in t]


# In[4]:


sa = [sa1,sa2,sa3]
ca = [ca1,ca2,ca3]
d_list = [d1, d2, d3]
a_list = [a1,a2,a3]
alpha_list = [alpha1, alpha2, alpha3]
cos_alpha_list = [cos(ii) for ii in ca]
sin_alpha_list = [sin(ii) for ii in sa]


# In[5]:


identities = [c[i] ** 2+s[i] ** 2 - 1 for i in range(3)]
identities_short = [c[i] ** 2+s[i] ** 2 - 1 for i in range(1, 3)]


# In[6]:


def my_subs(expression, symbols_existing, symbols_substitute):
    for ii, jj in zip(symbols_existing, symbols_substitute):
        # print(f"substituting {ii} to {jj}")
        expression = expression.subs(ii, jj) 
    return expression

def to_trig(expression):
    cos_substitution = my_subs(expression, c, cos_list)
    return my_subs(cos_substitution, s, sin_list)

def from_trig(expression):
    cos_substitution = my_subs(expression, cos_list, c)
    return my_subs(cos_substitution, sin_list, s)
    
def to_trig_alpha(expression):
    cos_substitution = my_subs(expression, ca, cos_alpha_list)
    return my_subs(cos_substitution, sa, sin_alpha_list)


# In[7]:


def z_translation_matrix(val):
    k = np.eye(4, 4)
    k[2, 3] = val
    return k


def x_translation_matrix(val):
    k = np.eye(4, 4)
    k[0, 3] = val
    return k


def z_rotation_matrix(in_theta):
    mat_id = eye(4, 4)
    mat_id[0, 0] = cos(in_theta)
    mat_id[1, 0] = sin(in_theta)
    mat_id[0, 1] = - sin(in_theta)
    mat_id[1, 1] = cos(in_theta)

    return mat_id


def x_rotation_matrix(in_theta):
    mat_id = np.eye(4, 4)
    mat_id[1, 1] = cos(in_theta)
    mat_id[2, 1] = sin(in_theta)
    mat_id[1, 2] = -sin(in_theta)
    mat_id[2, 2] = cos(in_theta)

    return mat_id


# In[8]:


def jacobian(array_diff):
    list_symbols = array_diff[0].free_symbols
    bool_matrix_initiation = False
    for item in array_diff:
        new_symbols = item.free_symbols
        for ii in new_symbols:
            list_symbols.add(ii)
    list_symbols = list(set(list_symbols))
    print(f"the list of symbols is {list_symbols}")
    for item in array_diff:
        array_jacobian = [diff(item, symbol_diff) for symbol_diff in list_symbols]
        if not bool_matrix_initiation:
            M1 = Matrix([array_jacobian])
            bool_matrix_initiation = True
        else:
            M1 = np.vstack((M1, array_jacobian))
    print(f"the shape of output matrix is {shape(M1)}")
    return Matrix(M1)


# In[9]:


def T_mat(theta_list, d_list, a_list, alpha_list):
    M1 = np.eye(4, 4)
    for index_i in range(len(d_list)):
        M1 = np.matmul(np.matmul(M1, z_rotation_matrix(theta_list[index_i])), z_translation_matrix(d_list[index_i]))
        M1 = np.matmul(np.matmul(M1, x_rotation_matrix(alpha_list[index_i])), x_translation_matrix(a_list[index_i]))

    return M1


# In[10]:


theta_list = t
d_list = [0, 1, 0]
a_list = [1, 2, 3 / 2]
alpha_list = [-pi / 2, pi / 2, 0]


# In[11]:


def give_determinant(theta_list, d_list, a_list, alpha_list):
    T_ee = T_mat(theta_list, d_list, a_list, alpha_list)
    x_cart = from_trig(T_ee[0, 3]) 
    y_cart = from_trig(T_ee[1, 3]) 
    z_cart = from_trig(T_ee[2, 3]) 
    vector_translation = T_ee[0:3, 3]
    return simplify(det(jacobian(vector_translation)))


# In[12]:


determinant = give_determinant(t, d_list, a_list, alpha_list)


# In[13]:


T_ee = T_mat(theta_list, d_list, a_list, alpha_list)
x_cart = from_trig(T_ee[0, 3]) 
y_cart = from_trig(T_ee[1, 3]) 
z_cart = from_trig(T_ee[2, 3]) 


# In[14]:


x2y2 = simplify(T_ee[0, 3] ** 2 + T_ee[1, 3] ** 2)


# In[15]:


x2y2_cart = from_trig(x2y2)


# In[16]:


forward_kinematics = [x2y2_cart - rho ** 2, z_cart - z]


# In[17]:


det_eq = from_trig(determinant)


# In[18]:


forward_kinematics.append(det_eq)


# In[19]:


forward_kinematics.extend(identities_short)


# In[ ]:





# In[20]:


[print(ii.free_symbols) for ii in forward_kinematics]


# In[21]:


det_eq


# In[22]:


res_x2y2_c2 = resultant(forward_kinematics[0], forward_kinematics[3], c2)


# In[23]:


res_z_c2 = resultant(forward_kinematics[1], forward_kinematics[3], c2)


# In[24]:


res_det_c2 = resultant(forward_kinematics[2], forward_kinematics[3], c2)


# In[32]:


res_x2y2_z_s2 = resultant(res_x2y2_c2, res_det_c2, s2)


# In[36]:


res_x2y2_c2.is_algebraic_expr()


# In[30]:


res_det_c2


# In[ ]:




