classdef Item
	properties
		pose = eye(4);
		grabOrientationOffset = eye(4); %transform from item pose to ideal grabbing orientation
		grabPose = pose * grabOrientationOffset;
		grabWidth = 0; %width of jaw to grab
		itemBox = RectangularPrism([1,-1,-1], [1,1,1]);

		% optional properties
		weight %for checking/sorting
		type %for sorting
	end
	methods
		function p = setPose(obj, pose)
			obj.pose = pose;
            p = obj.pose;
		end
		function o = setGrabOrientationOffset(obj, grabOrientationOffset)
			obj.grabOrientationOffset = grabOrientationOffset;
            o = obj.grabOrientationOffset;
		end

		function p = getPose(obj)
			p = obj.pose;
		end
		function p = getGrapPose(obj)
			p = obj.grabPose;
		end
	end
end