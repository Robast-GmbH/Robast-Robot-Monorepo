import DisplayAnImage from './DisplayAnImage'
import TempMap from '../imgs/temp_quadrat.png'





function SimpleMap( ) {
        return (
                <div id="SimpleMap">
                      <img src={TempMap} draggable="false" alt="Robot_Map"/>  
                </div>
        )
}

SimpleMap.propTypes = {}

export default SimpleMap