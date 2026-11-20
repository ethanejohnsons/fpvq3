// fpvq3 procedural drone model

fpvq3/drone_solid
{
	nopicmip
	cull none
	{
		map $whiteimage
		rgbGen vertex
	}
}

fpvq3/drone_prop
{
	nopicmip
	cull none
	sort additive
	{
		map $whiteimage
		blendFunc GL_SRC_ALPHA GL_ONE_MINUS_SRC_ALPHA
		rgbGen vertex
		alphaGen vertex
	}
}
