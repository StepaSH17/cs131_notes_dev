---
title: Optical Flow
keywords:
order: 0
---

- [7.1 Optical Flow](#7.1-optical-flow)
- [7.2 Lukas-Kanade Method](#7.2-lukas-kanade method)
	- [7.2.1 Motivation for Lucas-Kanade](#7.2.1-motivation-for-lucas-kanade)
	- [7.2.2 Lucas-Kanade Flow](#7.2.2-lucas-kanade-flow)
	- [7.2.3 Interpreting the Eigenvalues](7.2.3-interpreting-the-eigenvalues)
	- [7.2.4 Improving Accuracy](7.2.4-improving-accuracy)
- [7.3 Pyramids for Large Motion](#7.3-pyramids-for-large-motion)
	- [7.3.1 Previous Assumptions and Large Motion](#7.3.1-previous-assumptions-and-large-motion) 
	- [7.3.2 Optical Flow Estimation with Pyramids](#7.3.2-optical-flow-estimation-with-pyramids) 
	- [7.3.3 Optical Flow Results](#7.3.3-optical-flow-results) 
- [7.4 Horn-Schunk Method](#7.4-horn-schunk-method)
- [7.5 Applications](#7.5-applications)

[//]: # (This is how you can make a comment that won't appear in the web page! It might be visible on some machines/browsers so use this only for development.)

[//]: # (Notice in the table of contents that [First Big Topic] matches #first-big-topic, except for all lowercase and spaces are replaced with dashes. This is important so that the table of contents links properly to the sections)

[//]: # (Leave this line here, but you can replace the name field with anything! It's used in the HTML structure of the page but isn't visible to users)
<a name='Topic 1'></a>

## 7.0 LEAVE THIS UNTIL WE ARE DONE so everyone has instructions
	
Here you can start to talk about the first topic of your notes. You can bold text like **this**, or italicize text like *this*. If you want to make a numbered list it's as easy as
1.  
2. 
3. 

- Bullet
- points
- are
- similar 

For a more detailed cheatsheet on the most important functionality of Markdown, check out this link https://wordpress.com/support/markdown-quick-reference/, which you can format in Markdown with your own [link title](https://wordpress.com/support/markdown-quick-reference/)


<a name='Subtopic 1-1'></a>
### Subtopic 1-1
You might want to include images in your notes, since Computer Vision as a field is blessed with tons of cool visualizations. Here's an example from the CS 231N notes page we included as a reference for you:

<div class="fig figcenter fighighlight">
  <img src="{{ site.baseurl }}/assets/examples/classify.png">
  <div class="figcaption">Put your informative caption here! If you really want to mess around with the classes in this div container then feel free, but inserting images just like this should work great!</div>
</div>

<a name='Subtopic 1-2'></a>
### Subtopic 1-2
Sometimes you might want to insert some code snippets into your notes. As an example, here's a snippet of python code taken from the CS 231N notes:
```python
Xtr, Ytr, Xte, Yte = load_CIFAR10('data/cifar10/') # a magic function we provide
# flatten out all images to be one-dimensional
Xtr_rows = Xtr.reshape(Xtr.shape[0], 32 * 32 * 3) # Xtr_rows becomes 50000 x 3072
Xte_rows = Xte.reshape(Xte.shape[0], 32 * 32 * 3) # Xte_rows becomes 10000 x 3072
```

<a name='Subtopic 1-3'></a>
### Subtopic 1-3
Sometimes you might want to write some mathematical equations, and LaTeX is a great tool for that! You can write an inline equation like this \\( a^2 = b^2 \\), or you can display an equation on its own line like this! \\[ a^2 = b^2 + c^2 \\]

You can also apply LaTeX syntax to label your equations and refer to them later! Here's the equation:

$$ \begin{equation} \label{your_label} a^2 = b^2 + c^2 + d^2 + e^2 \end{equation} $$

and here's a linked reference to it: \eqref{your_label}. For now, this configuration likes the \\"\\$\\$ equation stuff ... \\$\\$\\" syntax to have an empty line above and below it, but it displays the same anyway.

**For a guide on LaTeX syntax and how to write mathematical equations and formulas with it, check out [this link](https://www.overleaf.com/learn/latex/mathematical_expressions)** 

**Here's a short guide on how to use the basics of LaTeX**
- You've seen above the syntax to start and end an equation, so now let's work on what you fill in the middle
- You can make variables and expressions **bold** in equations too: \\(\mathbf{x} + y\\)
- Superscripts and subscripts are easy: use the ^ and _ symbol and bound your super/sub script by {} if it's more than one character. For example: \\(e^{-x+10}\\)
- Greek letters are also simple, use the \ character with their written name with optional capitalization, such as alpha or Alpha. For example: \\(\alpha + \beta + \gamma + \delta + \Gamma + \Delta\\). Not all capital greek letters work like this, but you can search online for solutions if this trick fails or reach out to the CA's. In general the \ character in LaTeX is the gateway to all kinds of special characters and functionalities.
- Sums and Products are really useful in Latex. You can use both superscripts and subscripts to mark the bounds: \\(\log(\prod_{i=0}^{2n}i^2) = \sum_{i=0}^{2n}\log (i^2)\\)
- Another useful trick is to write out a matrix or a vector in LaTex. There's a lot of customization you can do with this, so check out this [page](https://www.overleaf.com/learn/latex/Matrices) for more details. Here's some examples in our Markdown environment: 


$$\begin{bmatrix}
1 & 2 & 3\\
a & b & c
\end{bmatrix}$$

$$\begin{bmatrix}
1\\
2\\
3\\
\end{bmatrix}$$

$$\begin{bmatrix}
1 & 2 & 3\\
\end{bmatrix}$$

As with the labelled equations, it makes a difference whether the lines above and below the equation are blank, so keep that in mind while debugging! 

## 7.1 Optical Flow
This should give you the primary tools to develop your notes. Check out the [markdown quick reference](https://wordpress.com/support/markdown-quick-reference/) for any further Markdown functionality that you may find useful, and reach out to the teaching team on Piazza if you have any questions about how to create your lecture notes


By leveraging the brightness constancy assumption and the previous derivation, an equation that relates the gradient of I, u, v, and I_t(Image derivative along t) is obtained:

$$ \begin{equation} \nabla I \cdot \begin{bmatrix} u & v \end{bmatrix}^T + I_t = 0 \end{equation} $$

Note this single equation by itself cannot recover image motion (u, v) since there are two unknowns.

However, we can leverage the assumption of spatial coherence that states neighboring pixels move similarly so it can be assumed  u, v values for a pixel should be identical to u, v values of its neighboring pixels to obtain additional equations to solve for u, v. This is discussed later.

Note a constraint of the brightness constancy assumption is that the component of the flow perpendicular to the gradient, which is parallel to the edge, cannot be measured because equation above is underdetermined.

This can be more easily seen via the diagram below and the following analysis considering:
- If (u, v) satisfies the equation above from brightness constancy assumption, then \\( \nabla I \cdot \begin{bmatrix} u & v \end{bmatrix}^T + I_t = 0 \\)
- Choose (u', v') is perpendicular to gradient of I, then \\( \nabla I \cdot \begin{bmatrix} u' & v' \end{bmatrix}^T = 0 \\)
- Summing the two equations above yield: \\( \nabla I \cdot \begin{bmatrix} u+u' & v+v' \end{bmatrix}^T + I_t = 0 \\) demonstrating (u+u', v+v') also satisfies the equation above regardless of what (u', v') actually are as long as (u', v') are parallel to edge.

<div class="fig figcenter fighighlight">
  <img src="{{ site.baseurl }}/videos/picture_7.1/Brightness_constancy_constraint.PNG">
</div>

As a result, the component of (u,v) that is parallel to the gradient can be measured well, but the component of (u,v) that is perpendicular to the gradient (parallel to edge) cannot be measured with confidence.

This constraint can also be seen visually when considering "The aperature problem" as demonstrated in the figure below where a downward ramp moves identically when the entire ramp can be seen and when part of the ramp (outside of the circle) is masked. Although the ramp is moving identically, the perceived motion (when masking view outside the circle) differs from the actual motion. This demonstrates we can only visually see the motion of the edge perpendicular to the edge because we're only measuring motion using a finite, small neighborhood, which again is called "The aperature problem".

<div class="fig figcenter fighighlight">
  <img src="{{ site.baseurl }}/videos/picture_7.1/aperture_problem.PNG">
</div>

This problem is also seen in the barber pole illusion. 

## 7.2 Lukas-Kanade Method

 ### 7.2.1 Motivation for Lucas-Kanade

 From 7.1, we were left with some ambiguity: we have 2 unknowns (u, v) at every pixel in the image, where u(x, y) represents x-direction motion and v(x, y) represents y-direction motion. Specifically, we have the equation:

 $$ \nabla{I} \cdot 
 \begin{bmatrix} 
 u\\ 
 v\\ 
 \end{bmatrix} 
 + I_t = 0 $$

 Where \\( \nabla{I} \\) is the intensity gradient of the image with respect to x and y, and \\( I_t \\) is the intensity gradient with respect to t, or the "temporal derivative." As you can see, we are underconstrained because we have one equation and two unknowns (the gradients are scalar).

 To uncover more constraint equations, we introduce the **spatial coherence contraint,** which assumes that pixel neighbors have the same optical flow value (u, v). This lets us redefine our previous equation as:

 $$ \nabla{I(p_{i})} \cdot 
 \begin{bmatrix} 
 u\\ 
 v\\ 
 \end{bmatrix} 
 + I_{t}(p_i) = 0 $$

 Where \\( \nabla{I(p_{i})} \\) is the intensity gradient at pixel i and \\( I_{t}(p_i) \\) is the gradient of I with respect to t at pixel i.

 Now we have an **overconstraining** problem, because if we use a window of size m x m to evaluate intensity, we now have m * m equations for only 2 unknowns. For example, given a 5 x 5 window, we have 25 equations for 2 unknowns, and our sequence of equations is:

 $$ 
 \begin{bmatrix} 
 I_{x}(p_1) & I_{y}(p_1)\\ 
 ... & ...\\
 I_{x}(p_{25}) & I_{y}(p_{25})\\
 \end{bmatrix}
 \cdot
 \begin{bmatrix} 
 u\\ 
 v\\ 
 \end{bmatrix}
 =
 -\begin{bmatrix}
 I_{t}(p_1)\\
 ...\\
 I_{t}(p_{25})\\
 \end{bmatrix}
 $$

 In 7.2.2, we will resolve this overconstraint issue.

 ### 7.2.2 Lucas-Kanade Flow
 By leveraging the brightness constancy assumption and the previous derivation, an equation that relates the gradient of I, u, v, and I_t(Image derivative along t) is obtained:

 In 7.2.1, we showed an example where we were overconstrained with 25 equations solving 2 unknowns. To resolve this issue, we will use least-squares to solve for the vector \\(\begin{bmatrix} u\\ v\\ \end{bmatrix} \\).
 $$ \begin{equation} \nablaI \cdot \begin{bmatrix} u & v \end{bmatrix}^T + I_t = 0 \end{equation} $$

 Referring to our previous system of equations of form \\( A \cdot d = b \\), we will instead use the least-squares form \\( (A^{T}A)d = A^{T}b \\). This new system of equations is equivalent to:
 Note this single equation by itself cannot recover image motion (u, v) since there are two unknowns.

 $$
 \begin{bmatrix}
 \Sigma I_xI_x & \Sigma I_xI_y\\
 \Sigma I_xI_y & \Sigma I_yI_y\\
 \end{bmatrix} 
 \cdot 
 \begin{bmatrix}
 u\\
 v\\
 \end{bmatrix} =
 -\begin{bmatrix}
 \Sigma I_xI_t\\
 \Sigma I_yI_t\\
 \end{bmatrix}
 $$
 However, we can leverage the assumption of spatial coherence that states neighboring pixels move similarly so it can be assumed  u, v values for a pixel should be identical to u,v values of its neighboring pixels to obtain additional equations to solve for u, v. This is discussed later.

 The dimensions of resulting matrix are (2 x 2) x (2 x 1) = 2 x 1, exactly what we wanted.
 Note a constraint of the brightness constancy assumption is that the component of the flow perpendicular to the gradient, which is parallel to the edge, cannot be measured because equation is underdetermined.

 Some requirements for implementing Lucas-Kanade:
 This can be more easily seen via the diagram below and the following analysis considering:
 - If (u, v) satisfies the equation above from brightness constancy assumption, then \\( \nablaI \cdot \begin{bmatrix} u & v \end{bmatrix}^T + I_t = 0 \\)
 - Choose (u', v') is perpendicular to gradient of I, then \\( \nablaI \cdot \begin{bmatrix} u' & v' \end{bmatrix}^T = 0 \\)
 - Summing the two equations above yield: \\( \nablaI \cdot \begin{bmatrix} u+u' & v+v' \end{bmatrix}^T + I_t = 0 \\) demonstrating (u+u', v+v') also satisfies the equation above regardless of what (u', v') actually are as long as (u', v') are parallel to edge.

 - \\( A^TA \\) should be invertible
 - \\( A^TA \\) should be large enough to minimize noise (e.g. eigenvalues \\( \lambda_1 \\) and \\( \lambda_2 \\) should be not too small
 - \\( A^TA \\) should be well-conditioned (e.g. \\( \lambda_1/ \lambda_2 \\) not too large for \\( \lambda_1 > \lambda_2 \\)

 What does this matrix \\( A^TA \\) remind you of?

 That's right, the second moment matrix M from the Harris corner detector! Both deal with eigenvalues and a 2 x 2 matrix of summations of gradients of an image.

 Note that the eigenvectors and eigenvalues of \\( A^TA \\) determine the edge direction and magnitude:

 - The eigenvector with the larger eigenvalue contains the direction of fastest intensity change
 - The other eigenvector is perpendicular to the first

 ### 7.2.3 Interpreting the Eigenvalues

 As noted in 7.2.2, eigenvalues should have a certain size and proportion to each other in order to provide reliable corner detection. The following image outlines the effects of each eigenvalue scenario and their pitfalls.
 As a result, the component of (u,v) that is parallel to the gradient can be measured well, but the component of (u,v) that is perpendicular to the gradient (parallel to edge) cannot be measured with confidence.

 <div class="fig figcenter fighighlight">
   <img src="{{ site.baseurl }}/videos/picture_7.3/LK_eig_interpretation.png">
   <img src="{{ site.baseurl }}/videos/picture_7.1/Brightness_constancy_constraint.png">
 </div>

 1. **Edges:** Around edges, there is an aperture problem. We can measure optical flow in the direction perpendicular to the edge, but along the edge it is hard to discern change.
 2. **Flats:** Flat areas are regions of low texture. These are challenging because of their homogeneity and are not easily distinguishable.
 3. **Corners:** Corners are the regions we would like to capture in our Lucas-Kanade study. These provide relatively large eigenvalues for each eigenvector, and are relatively distinguishable features.

 ### 7.2.4 Improving Accuracy

 In this section, we will evaluate areas for accuracy improvement.

 Recall the small motion assumption from 7.1:

 $$
 0 = I(x + u, y + v) - I_t(x, y) \approx I(x, y) + I_xu + I_yv - I_t(x, y)
 $$

 This is an approximation, but if we add back higher order terms, we can achieve even greater accuracy:
 This constraint can also be seen visually when considering "The aperature problem" as demonstrated in the figure below where a decreasing ramp moves identically when the entire ramp can be seen and when part of the ramp (outside of the circle) is masked. Although the ramp is moving identically, the perceived motion (when masking view outside the circle) differs from the actual motion. This demonstrates we can only visually see motion of the edge perpendicular to the edge because we're only measuring motion using a finite, small neighborhood, which again is called "The aperature problem".

 \\( I(x, y) + I_xu + I_yv + \\) higher order terms \\( - I_t(x, y)\\)

 The process for finding these higher terms is a polynomial root finding problem. This can be approached in a few ways:

 1. Using Newton's method (out of the scope of this class)
 2. More iterations of Lucas-Kanade (increases computational complexity)

 **Iterative Refinement:**

 Another approach to improving accuracy:

 1. Estimate the velocity at each pixel by solving the Lucas-Kanade equations (find (u, v))
 2. Warp the previous image (I(t - 1)) toward the next image (I(t)) using the estimated flow field found in Step 1. This creates a predicted next frame based on (u, v).
 3. Repeat until convergence (this handles residuals not captured in Step 2)

 **Potential Sources of Error in Lucas-Kanade:**

 Recall that in order to perform Lucas-Kanade, we made a few assumptions:

 - Assumed \\( A^TA \\) is easily invertible
 - Assumed there is not much noise (eigenvalues of \\( A^TA \\) not too small)
 <div class="fig figcenter fighighlight">
   <img src="{{ site.baseurl }}/videos/picture_7.1/aperature_problem.png">
 </div>

 When these assumptions are violated,
 This problem is also seen in the barber pole illusion. 

 - Our small motion assumption is invalid (see 7.1 and 7.2.4)
 - Brightness constancy is not satisfied (can't rely on intensity values)
 - A given pixel doesn't necessarily move like its neighbors (window size too large)

## 7.3 Pyramids for Large Motion
### 7.3.1 Previous Assumptions and Large Motion
We made some key assumptions in the previous section about Lukas-Kanade Method, such as: 

**Small motion** - points do not move very far

**Brightness constancy** - projection of the same point is the same in every frame

**Spacial coherence** - points move like their neighbors

However we still need to consider cases with larger motion, more than one pixel. One potential idea that can help us is reducing resolution of the video, such that motion size is one pixel. 
<div class="fig figcenter fighighlight">
  <img src="{{ site.baseurl }}/videos/picture_7.3/IMG_3774.jpg">
</div>
Using that we can detect optical flow at the lower resolution image, and use that to build up back to higher resolution video. 

### 7.3.2 Optical Flow Estimation with Pyramids
We want to calculate optical flow in a coarse-to-fine approach. We first calculate the pyramid of the image, by converting image in lower resolution versions in multiple layers. As a result, motion that was 10 pixels in the high-resolution image, results in motion of 1.25 pixels in a lower resolution version. 
<div class="fig figcenter fighighlight">
  <img src="{{ site.baseurl }}/videos/picture_7.3/IMG_BECD9E44C44F-1.jpeg">
</div>

We run Lukas-Kanade on the low resolution image, because we can do it as now we satisfy small motion requirment. We use solution of that algorithm to upsample to the next layer with a little higher resolution. There we use L-K again but to detect residual movements that were not detected as a result of resolution decrease. We keep repeating that process until we get back to the original high resolution layer. 
<div class="fig figcenter fighighlight">
  <img src="{{ site.baseurl }}/videos/picture_7.3/IMG_AEBE3D9581A9-1.jpeg">
</div>

### 7.3.3 Optical Flow Results
We can see what happens when we don't use pyramids. Vectors in areas of large motion are very innacurate and fail to represent reality. 
<div class="fig figcenter fighighlight">
  <img src="{{ site.baseurl }}/videos/picture_7.3/IMG_78D6637B7B4A-1.jpeg">
</div>
Whereas on the picture where we use pyramids, we can see that vectors are properly alligned and calculated. 
<div class="fig figcenter fighighlight">
  <img src="{{ site.baseurl }}/videos/picture_7.3/IMG_8F39FB500A2B-1.jpeg">
</div>

## 7.4 Horn-Schunk Method
The Horn-Shunk method seeks to formulate optical flow as a global energy function minimization. So, we will minimize an energy function for an entire image. We can represent a global energy function with the following equation: \\[ E = \int \int (I_xu+ I_yv + I_t)^2 + \alpha^2 (||\delta u||^2 + ||\delta v||^2)] dx dy \\]
 There are 3 major components of this equation to consider during our minimization process. \newline  
 1. The 
 \\((I_xu+ I_yv + I_t)^2 \\) component of the equation represents the brightness constancy, which we want to be equal to 0 in order to minimize our energy function. Thus, we want to calculate the values for u and v that minimize this term. \newline 
 2. The \\(||\delta u||^2 + ||\delta v||^2) \\) component of the equation is the smoothness constant. By measuring the magnitude of the the gradient of u and the gradient of v, we are verifying that the changes between pixels are small. 
 \newline 
 3. \\(\alpha \\) is the regularization constant. A larger alpha enforces our smoothness constraint more than the brightness constraint. 

 We can solve this minimization problem by deriving the equation with respect to \\( u\\) and \\( v\\) to get the following equations and can solve for u and v: 

 \\[ 0 = I_x(I_xu + I_yv + I_t) - \alpha \Delta u \\]
 \\[ 0 = I_y(I_xu + I_yv + I_t) - \alpha \Delta v \\]
 In these equations, \( \Delta \) is the LaGrange operator, which can be measured using the following equation: 
 \\[ \Delta u(x, y) = \bar{u} (x,y) - u(x, y) \\]
 Where \\( \bar{u} \\) is the weighted average of u measured at a neighborhood around (x, y). 

 We can substitute this definition for \\( \Delta \\) into our original 2 equations to get: 
 \\[ (I^2_x + \alpha^2)u + I_xI_yv = \alpha^2 \bar{u} - I_xI_t \\]
 \\[ (I^2_y + \alpha^2)v + I_xI_yu = \alpha^2 \bar{v} - I_yI_t \\]
 These equations are linear in both u and v, so they can be solved analytically for each pixel individually. However, notice that the solution for \\( u \\) and  \\( v \\) for a pixel (x, y) depends on the optical flow values in a neighborhood around a (x, y), so we must iteratively update u and v once their neighbors have been updated. We get the following recursive equation to show this relationship: 
 \\[ u^{k + 1} = \bar{u}^k - \frac{I_x(I_x\bar{u}^k + I_y\bar{v}^k + I_t)}{\alpha^2 + I_x^2 + I_y^2}\\]
 \\[ v^{k + 1} = \bar{v}^k - \frac{I_y(I_x\bar{u}^k + I_y\bar{v}^k + I_t)}{\alpha^2 + I_x^2 + I_y^2}\\]
 where we define \\( \bar{u}^k\\) and \\(\bar{v}^k\\) as the kth values of \\(\bar{u}\\) and \\(\bar{v}\\). 

 The smoothness regularization constant is a sum of squared terms that we place in the expression to be minimized. It's important to note that in texture-free regions, there is no optical flow and on edges, points will flow to the nearest point, which solves the aperture problem. 


 Michael Black extended the Horn-Schunk method. He replaced the quadratic regularization function term: 
 <img width="297" alt="Screen Shot 2021-10-18 at 8 53 14 PM" src="https://user-images.githubusercontent.com/68766243/137841132-1b6adfeb-848b-4775-8f33-379894c25686.png">
 with this function: 
 <img width="302" alt="Screen Shot 2021-10-18 at 8 53 47 PM" src="https://user-images.githubusercontent.com/68766243/137841178-9a01add9-8349-4d4e-b6f9-42395fea01c8.png">


 This should give you the primary tools to develop your notes. Check out the [markdown quick reference](https://wordpress.com/support/markdown-quick-reference/) for any further Markdown functionality that you may find useful, and reach out to the teaching team on Piazza if you have any questions about how to create your lecture notes

## 7.5 Applications
 There are multiple things we can do with Motion Features once we estimate optical flow.

 Application Examples: </br>
 </br>
 &nbsp;&nbsp; 	**Estimating 3D Structure:** <br/>
 &nbsp; &nbsp;&nbsp;	Given an input sequence we can calculate how pixels move between frames and that can help us estimate the 3D structure of the scene. <br/>
 <br/>
 &nbsp;&nbsp;	**Segmenting Objects Based on Motion Cues:** <br/>
 <br/>
 &nbsp;&nbsp;&nbsp;       ***1. Background subtraction:*** <br/>
 &nbsp;&nbsp;&nbsp;&nbsp; - In a fixed camera setting, for example, surveillance, you may want to identify the pixels that belong to the background scene. One way to do that is by calculating optical flow and deciding which pixels never move in the scene. Those may correspond to the background of the scene. <br/>
 &nbsp;&nbsp;&nbsp;&nbsp; - So assuming that we have a static camera that is observing the scene, our goal here would be to separate the static background from the moving foreground. <br/>
 <br/>
 &nbsp;&nbsp;&nbsp;  	***2. Motion Segmentation:*** <br/>
 &nbsp;&nbsp;&nbsp;&nbsp;		- We can also use motion estimation to perform segmentation of video sequences. The ideas is to divide the video into multiple coherently moving objects.<br/>
 <br/>
 &nbsp;&nbsp;&nbsp; 	***3. Tracking Objects:***<br/>
 &nbsp;&nbsp;&nbsp;&nbsp;		- We can also perform tracking, where we’re trying to follow an object as it moves through the video. <br/>
 &nbsp;&nbsp;&nbsp;&nbsp;		- For example in this traffic scene, we are trying to follow this care is it moves through the highway. <br/>
 <br/>
 &nbsp;&nbsp;&nbsp;  	***4. Synthetic Dynamic Textures:*** <br/>
 &nbsp;&nbsp;&nbsp;&nbsp;		- The idea is that you’re given a short clip of a moving texture, and you want to produce a longer clip that replicates the original motion <br/>
 <br/>
 &nbsp;&nbsp;   **Super-Resolution** <br/>
 &nbsp;&nbsp;&nbsp;          - The ideas of motion can also be applied to super-resolution. We start with a set of low-quality images of the same scene. By estimating how pixels may move from one image to the other we can actually recover a higher-resolution version of the image. <br/>
 <br/>
 &nbsp;&nbsp;   **Recognizing events and activities** <br/>
 &nbsp;&nbsp;&nbsp;     	- We can estimate a persons body position and then try to analyze how the body moves to try to recognize the type of activity the person is performing.<br/>
 &nbsp;&nbsp;&nbsp; 	- In his own research, he has used motion to try to recognize human actives, for example, different types of speed in figure skating. We can also try to recognize events that happen in groups of people. (i.e. Crossing, Talking, Queuing, Dancing, Jogging) <br/>
 <br/>
 &nbsp;&nbsp;    **Human Event Understanding: From Actions to Tasks** <br/>
 &nbsp;&nbsp;&nbsp;    	- http://tv.vera.com.uy/video/55276 <br/>
 <br/>
 &nbsp;&nbsp;     **Optical Flow without Motion** <br/>
 &nbsp;&nbsp;&nbsp;       - **Fun Fact: As humans we can sometimes perceive motion even when the object is static.** <br/>
