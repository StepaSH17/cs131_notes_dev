---
title: Optical Flow
keywords:
order: 0
---

- [7.1 Optical Flow](#7.1-optical-flow)
- [7.2 Lukas-Kanade Method](#7.2-lukas-kanade method)
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

## 7.2 Lukas-Kanade Method
This should give you the primary tools to develop your notes. Check out the [markdown quick reference](https://wordpress.com/support/markdown-quick-reference/) for any further Markdown functionality that you may find useful, and reach out to the teaching team on Piazza if you have any questions about how to create your lecture notes

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
