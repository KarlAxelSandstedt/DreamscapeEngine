#version 300 es

precision mediump float;

in vec4 out_color;

out vec4 fragColor;

void main()
{
	fragColor = out_color;
}
