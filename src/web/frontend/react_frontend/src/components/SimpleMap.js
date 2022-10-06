
import TempMap from '../imgs/RobastVisitenkarte2.jpg'





function SimpleMap( ) {
        return (
                <div id="SimpleMap">
                      <img src={TempMap} draggable="false" alt="Robot_Map"/>  
                </div>
        )
}

SimpleMap.propTypes = {}

export default SimpleMap