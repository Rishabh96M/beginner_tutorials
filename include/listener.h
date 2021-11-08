/**
 * Copyright (c) 2021 Rishabh Mukund
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef INCLUDE_LISTENER_H_
#define INCLUDE_LISTENER_H_


#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
/**
 * @brief The callback function for the subscriber
 * 
 * @param msg message received
 */

void pubSubCallback(const std_msgs::String::ConstPtr& msg);

#endif  // INCLUDE_LISTENER_H_
