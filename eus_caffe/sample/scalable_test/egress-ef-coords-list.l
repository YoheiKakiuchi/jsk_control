(setq *ef-coords-list*
      (let* ((larm0
	      (instance vector-coords :init (send (nth 0 *ef-coords*) :copy-worldcoords)))
	     (rarm0
	      (instance vector-coords :init (send (nth 1 *ef-coords*) :copy-worldcoords)))
	     (lleg0
	      (instance vector-coords :init (send (nth 2 *ef-coords*) :copy-worldcoords)))
	     (caffe-rleg0 (send (send *root-link* :worldcoords) :copy-worldcoords))
	     (ik-rleg0 (send (send (send *robot* :get :root-link) :worldcoords)
			     :copy-worldcoords)))
	(list
	 ;;
	 ;; init pose
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 ;;
	 ;; move lleg
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 0 30)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 80 0)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 0 -30)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 ;;
	 ;; move rleg
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 0 30) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 0 30) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 80 0) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 80 0) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 0 -30) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 0 -30) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0
	       )
	 ;;
	 ;; move larm
	 (list :ef-coords
	       (list (setq larm0
			   (send larm0 :transform-copy (make-coords :pos (float-vector 0 0 30))))
		     rarm0 lleg0)
	       :support-limbs '(:rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list (setq larm0
			   (send larm0 :transform-copy (make-coords :pos (float-vector 0 80 0))))
		     rarm0 lleg0)
	       :support-limbs '(:rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list (setq larm0
			   (send larm0 :transform-copy (make-coords :pos (float-vector 0 0 -30))))
		     rarm0 lleg0)
	       :support-limbs '(:rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 ;;
	 ;; move rarm
	 (list :ef-coords
	       (list larm0
		     (setq rarm0
			   (send rarm0 :transform-copy (make-coords :pos (float-vector 0 0 30))))
		     lleg0)
	       :support-limbs '(:larm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0
		     (setq rarm0
			   (send rarm0 :transform-copy (make-coords :pos (float-vector 0 80 0))))
		     lleg0)
	       :support-limbs '(:larm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0
		     (setq rarm0
			   (send rarm0 :transform-copy (make-coords :pos (float-vector 0 0 -30))))
		     lleg0)
	       :support-limbs '(:larm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 ;; move lleg
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 0 30)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 80 0)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 0 -30)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 ;;
	 ;; move rleg
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 0 30) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 0 30) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 80 0) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 80 0) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 0 -30) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 0 -30) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0
	       )
	 ;;
	 ;; move larm
	 (list :ef-coords
	       (list (setq larm0
			   (send larm0 :transform-copy (make-coords :pos (float-vector 0 0 30))))
		     rarm0 lleg0)
	       :support-limbs '(:rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list (setq larm0
			   (send larm0 :transform-copy (make-coords :pos (float-vector 0 80 0))))
		     rarm0 lleg0)
	       :support-limbs '(:rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list (setq larm0
			   (send larm0 :transform-copy (make-coords :pos (float-vector 0 0 -30))))
		     rarm0 lleg0)
	       :support-limbs '(:rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 ;;
	 ;; move rarm
	 (list :ef-coords
	       (list larm0
		     (setq rarm0
			   (send rarm0 :transform-copy (make-coords :pos (float-vector 0 0 30))))
		     lleg0)
	       :support-limbs '(:larm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0
		     (setq rarm0
			   (send rarm0 :transform-copy (make-coords :pos (float-vector 0 80 0))))
		     lleg0)
	       :support-limbs '(:larm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0
		     (setq rarm0
			   (send rarm0 :transform-copy (make-coords :pos (float-vector 0 0 -30))))
		     lleg0)
	       :support-limbs '(:larm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 ;;
	 ;; move lleg
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 0 30)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 80 0)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 0 -30)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 ;;
	 ;; move rleg
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 0 30) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 0 30) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 80 0) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 80 0) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 0 -30) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 0 -30) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0
	       )
	 ;;
	 ;; edge
	 ;;
	 ;;
	 ;;
	 ;; move rarm
	 (list :ef-coords
	       (list larm0
		     (setq rarm0
			   (send rarm0 :transform-copy (make-coords :pos (float-vector 0 0 30))))
		     lleg0)
	       :support-limbs '(:larm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0
		     (setq rarm0
			   (send rarm0 :transform-copy (make-coords :pos (float-vector 0 80 0))))
		     lleg0)
	       :support-limbs '(:larm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0
		     (setq rarm0
			   (send rarm0 :transform-copy (make-coords :pos (float-vector 0 0 -30))))
		     lleg0)
	       :support-limbs '(:larm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 ;;
	 ;; move lleg
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 0 30)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 80 0)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 0 -30)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 ;;
	 ;; move rleg
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 0 30) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 0 30) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 80 0) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 80 0) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 0 -30) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 0 -30) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0
	       )
	 ;;
	 ;;
	 ;; move rarm
	 (list :ef-coords
	       (list larm0
		     (setq rarm0
			   (send rarm0 :transform-copy (make-coords :pos (float-vector 0 0 30))))
		     lleg0)
	       :support-limbs '(:larm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0
		     (setq rarm0
			   (send rarm0 :transform-copy (make-coords :pos (float-vector 0 80 0))))
		     lleg0)
	       :support-limbs '(:larm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0
		     (setq rarm0
			   (send rarm0 :transform-copy (make-coords :pos (float-vector 0 0 -30))))
		     lleg0)
	       :support-limbs '(:larm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 ;;
	 ;; move lleg
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 0 30)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 80 0)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 0 -30)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 ;;
	 ;; move rleg
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 0 30) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 0 30) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 80 0) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 80 0) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 0 -30) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 0 -30) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0
	       )
	 ;;
	 ;; back
	 ;;
	 ;;
	 ;;
	 ;; move lleg
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 0 30)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector -200 0 0)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0
		     (setq lleg0 (send lleg0 :transform-copy
				       (make-coords :pos (float-vector 0 0 -30)))))
	       :support-limbs '(:larm :rarm :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 (list :ef-coords
	       (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 ;;
	 ;; move rleg
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 0 30) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 0 30) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector -200 0 0) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector -200 0 0) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg)
	       :caffe-root-link
	       (setq caffe-rleg0 (send (send caffe-rleg0 :copy-worldcoords)
				       :translate (float-vector 0 0 -30) :world))
	       :ik-root-link
	       (setq ik-rleg0 (send (send ik-rleg0 :copy-worldcoords)
				    :translate (float-vector 0 0 -30) :world)))
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :support-limbs '(:larm :rarm :lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0
	       )
	 ;;
	 ;;
	 ;; move centroid on foot
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :user-valid-check #'(lambda (cog) (< (abs (+ 200 (aref cog 0))) 20))
	       :support-limbs '(:lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0
	       )
	 (list :ef-coords (list larm0 rarm0 lleg0)
	       :user-valid-check #'(lambda (cog) (< (abs (+ 200 (aref cog 0))) 20))
	       :support-limbs '(:lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0
	       )
	 (list :ef-coords
	       (list (setq larm0
			   (send larm0 :transform-copy
				 (make-coords :pos (float-vector -200 (* 3 80) 100))))
		     (setq rarm0
			   (send rarm0 :transform-copy
				 (make-coords :pos (float-vector -200 80 100))))
		     lleg0)
	       :user-valid-check #'(lambda (cog) (and (< (abs (- -200 (aref cog 0))) 20)
						      (< (abs (- 500 (aref cog 1))) 20)))
	       :support-limbs '(:lleg :rleg)
	       :caffe-root-link caffe-rleg0
	       :ik-root-link ik-rleg0)
	 )))