%% Robot Controll, structure only so far...


classdef Item
	properties
		pose = eye(4);
		grabOrientationOffset = eye(4); %transform from item pose to ideal grabbing orientation
		grabPose = pose * grabOrientationOffset;
		grabWidth = 0; %width of jaw to grab
		itemBox = rect........................................................

		% optional properties
		weight %for checking/sorting
		type %for sorting
	end
	methods
		function setPose(obj, pose)
			obj.pose = pose;
		end
		function setGrabOrientationOffset(obj, grabOrientationOffset)
			obj.grabOrientationOffset = grabOrientationOffset;
		end

		function p = getPose(obj)
			p = obj.pose
		end
		function p = getGrapPose(obj)
			p = obj.grabPose
		end
	end
end

classdef Bag
	properties
		pose = eye(4);
		internalBox = rect......
	end
end


function bagItem(robot, item, bag)
	%robot approach item
		%determine next pose
		
		%check collisions

		%move
		while moving
			%check collisions (<5cm?)
			%check errors (e-stop etc)
		end
	%robot grab item

	%robot move to safe pose

	%robot approach bag

	%robot place item

	%robot move to safe pose

end
