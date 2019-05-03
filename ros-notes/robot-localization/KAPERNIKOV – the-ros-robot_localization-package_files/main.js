
//GLOBAL VARS
var mediaBreakpoint = 1024;
var shrinkFactor = .7;
var growFactor = 1.3;
var isWindowBig = window.matchMedia('(max-width:' + (mediaBreakpoint-1) + 'px )').matches;
var maxScroll = 300;


jQuery(window).load(function(){
})


jQuery(document).ready(function(){

  openNieuwsbriefOnClick();

	//CHANGE GIF SIZE TO PREVENT SQUISHING
	if(window.matchMedia('(max-width:' + (mediaBreakpoint-1) + 'px )').matches){
		jQuery('.imgContainer picture').each(function(){
			jQuery(this).find('img').css({
			'width': '450px',
			'height': '450px',
			})
		})

	}

	//resize header BG's if on mobile
	if(window.matchMedia('(max-width:' + (mediaBreakpoint - 1) + 'px )').matches){
		resizeHeaderImage();
	}


	var prevWindowSize = jQuery(window).width();


	//ADD MENU TOGGLE LISTENER
	jQuery('#mainMenuToggler').click(function(event){
		var e = event || window.event;

		toggleMainMenu();
	})


	//PRELOAD GIFS
	var gifs = getGifs();
	var image = [];

	jQuery.each(gifs, function(index){
		image[index] = new Image();
		image[index].src = gif[index];
	})



	if(window.matchMedia('(min-width:' + mediaBreakpoint + 'px )').matches){

		jQuery(this).find('.imgContainer picture img').css({
			'width': '675px',
			'height': '675px',
		})

	}


	//ADD MOUSENTER LISTENER TO LOAD GIF
	jQuery('article').mouseenter(function(){
		if(jQuery(this).find('.imgContainer picture').attr('data-hasPlayed') != "yes"){
			var src = jQuery(this).find('.imgContainer picture').data('animation')
			jQuery(this).find('.imgContainer picture source').attr('srcset',src);
			jQuery(this).find('.imgContainer picture').attr('data-hasPlayed','yes');
		}

		if(window.matchMedia('(max-width:' + (mediaBreakpoint-1) + 'px )').matches){

			jQuery(this).find('.imgContainer picture img').css({
				'width': '450px',
				'height': '450px',
			})

		}


		if(window.matchMedia('(min-width:' + mediaBreakpoint + 'px )').matches){

			jQuery(this).find('.imgContainer picture img').css({
				'width': '675px',
				'height': '675px',
			})

		}

	})


	//ADD SMOOTHSCROLL
	var links = getLinks();
	jQuery.each(links, function(index){
		jQuery(this).click(function(event){
			var e = event || window.event;
			e.preventDefault();

			var id = jQuery(this).attr('href');

			jQuery('html, body').animate({
				scrollTop : jQuery(id).offset().top
			}, 500)
		})
	});


	//IE11 FLEXBOX FALLBACK
	// if(navigator.userAgent.indexOf('Trident') > -1){

		adjustImgMarginIe();

		jQuery(window).resize(function(){
			adjustImgMarginIe();
		});
	// }


	//ADD HEADERBANNER FADEOUT EFFECT ON SCROLL
	jQuery(window).scroll(function(){
		if(window.matchMedia('(min-width:' + mediaBreakpoint + 'px )').matches){

			if (jQuery(window).scrollTop() < maxScroll){
				opacity = jQuery(window).scrollTop() / maxScroll;

				jQuery('.headerBannerOverlay').css({
					'opacity':opacity,
				});
			} else{
				jQuery('.headerBannerOverlay').css({
					'opacity':1,
				});
			}
		}
	})


	//CHANGE GIF SIZE IF MEDIA BREAKPOINT CHANGES
	jQuery(window).resize(function(){
		if(window.matchMedia('(max-width:' + (mediaBreakpoint-1) + 'px )').matches){

			if(isWindowBig){

				jQuery(".imgContainer picture").each(function(index){


					var src = jQuery(this).data('animation');

					var width = parseInt(jQuery(this).find('img').css('width'));
					var height = parseInt(jQuery(this).find('img').css('height'));

					jQuery(this).find('img').css({
						'width': '450px',
						'height': '450px',
					})


				});


				resizeHeaderImage("small");

				isWindowBig = false;

			}


		}


		if(window.matchMedia('(min-width:' + mediaBreakpoint + 'px )').matches){

			if(!isWindowBig){
				jQuery(".imgContainer picture").each(function(index){

					var src = jQuery(this).data('animation');

					var width = parseInt(jQuery(this).find('img').css('width').split("px")[0]);
					var height = parseInt(jQuery(this).find('img').css('height').split("px")[0]);



					width = width*growFactor;
					height = height*growFactor;


					jQuery(this).find('img').css({
						'width': '675px',
						'height': '675px',
					})

				});
				resizeHeaderImage("big");

				isWindowBig = true;

			}

		}
	});

})


function adjustImgMarginIe(){
	if(window.matchMedia('(min-width:' + mediaBreakpoint + 'px )').matches){
		jQuery('.imgLeft .imgContainer').each(function(index){

			var img = jQuery(this).find('picture img');
			var containerWidth = jQuery(this).width();
			var imgWidth = img.width();
			var negativeMargin = imgWidth - containerWidth;


			img.css({
				'margin-left': -negativeMargin + 'px',
			})
		})
	}

	if( window.matchMedia('(max-width:' + (mediaBreakpoint) + 'px )').matches ){
		jQuery('.imgContainer').each(function(index){

			var img = jQuery(this).find('picture img');
			var containerWidth = jQuery(this).width();
			var imgWidth = img.width();
			var negativeMargin = (imgWidth - containerWidth)*.5;


			img.css({
				'margin-left': -negativeMargin + 'px',
			})
		})
	}

}

function getGifs(){
	var gifs = [];

	jQuery('imgContainer picture').each(function(){
		var data = jQuery(this).data('animation');
		gifs.push(data);
	})

	return gifs;
}

/*
* show the main menu on toggler click
*/
function toggleMainMenu(){
	jQuery('#mainMenu').toggleClass('active');
	jQuery('#mainMenuToggler').toggleClass('active');
	jQuery('html').toggleClass('fixed');
}


//GET ALL ID LINKS
function getLinks(){
	var returnVal = [];

	jQuery('a').each(function(index){
    console.log(jQuery(this))

      if( jQuery(this).attr('href')[0] === "#" ){
        returnVal.push(jQuery(this));
      }

	});

	return returnVal;

}

function openNieuwsbriefOnClick(){
  var nieuwsbriefButton = document.querySelector('.nieuwsbriefButton');
  var nieuwsbriefForm = document.querySelector('.nieuwsbriefForm');

  nieuwsbriefButton.addEventListener("click", function(){
    nieuwsbriefForm.classList.toggle('nieuwsbriefOpen');
  });
}

function resizeHeaderImage(size){
		//ADJUST HEADER IMAGES FOR MOBILE
		var headerImgStyle = jQuery('.headerBanner').attr('style');
		var headerImgBg = headerImgStyle.split("(");
		headerImgBg = headerImgBg[1].split(")");
		headerImgBg = headerImgBg[0];
		headerImgBgPathArray = headerImgBg.split("/");
		headerImgBg = headerImgBgPathArray[headerImgBgPathArray.length - 1];
		headerImgBg = headerImgBg.split(".");
		if (size == "big"){
			headerImgBg[0] = headerImgBg[0].split("-600x600")[0];

		}else{
			headerImgBg[0] += "-600x600";
		}
		headerImgBgPathArray[headerImgBgPathArray.length - 1] = headerImgBg.join(".");
		headerImgBgPathArray = headerImgBgPathArray.join("/");

		jQuery('.headerBanner').attr('style', "background-image:url(" + headerImgBgPathArray + ");");
}
